use std::cmp::Reverse;
use float_cmp::approx_eq;
use itertools::Itertools;
use jagua_rs::collision_detection::hazards::HazardEntity;
use jagua_rs::entities::{Instance, Layout, PItemKey};
use jagua_rs::geometry::geo_traits::CollidesWith;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::{debug, info, warn};
use ordered_float::OrderedFloat;
use rand::prelude::{Distribution, IteratorRandom};
use rand_distr::Normal;
use slotmap::SecondaryMap;
use crate::config::ExplorationConfig;
use crate::FMT;
use crate::optimizer::separator::{Separator, SeparatorConfig};
use crate::sample::uniform_sampler::convert_sample_to_closest_feasible;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;

// Notes on is_locked flag: The changes in explore.rs protect the global disruption phase, but the local search (separation) must also respect the lock.

// === CHANGE START ===
// Flag to toggle the adaptive square resizing logic
const ENABLE_ADAPTIVE_SQUARE_RECOVERY: bool = true;
// === CHANGE END ===

/// Algorithm 12 from https://doi.org/10.48550/arXiv.2509.13329
pub fn exploration_phase(instance: &SPInstance, sep: &mut Separator, sol_listener: &mut impl SolutionListener,  term: &impl Terminator, config: &ExplorationConfig) -> Vec<SPSolution> {
    //let mut current_width = sep.prob.strip_width();
   
    // 1. Get the large height from your input (e.g., 5000.0)
    let start_size = sep.prob.instance.base_strip.fixed_height;
    
    // 2. Force the strip width to match this height immediately
    //    This creates a 5000x5000 square (because of Step 1)
    sep.change_strip_width(start_size, None);
    
    let mut current_width = start_size;
    let mut best_width = current_width;

    let mut feasible_sols = vec![sep.prob.save()];

    sol_listener.report(ReportType::ExplFeas, &feasible_sols[0], instance);
    info!("[EXPL] starting optimization with initial width: {:.3} ({:.3}%)",current_width,sep.prob.density() * 100.0);

    let mut infeas_sol_pool: Vec<(SPSolution, f32)> = vec![];

    while !term.kill() {
        // Attempt to separate the current layout
        let local_best = sep.separate(term, sol_listener);
        let total_loss = local_best.1.get_total_loss();

        if total_loss == 0.0 {
            // If successfully separated
            if current_width < best_width {
                info!("[EXPL] feasible solution found! (width: {:.3}, dens: {:.3}%)",current_width,sep.prob.density() * 100.0);
                best_width = current_width;
                feasible_sols.push(local_best.0.clone());
                sol_listener.report(ReportType::ExplFeas, &local_best.0, instance);
            }
            // Shrink the strip width and clear the infeasible solution pool
            let next_width = current_width * (1.0 - config.shrink_step);
            info!("[EXPL] shrinking strip by {}%: {:.3} -> {:.3}", config.shrink_step * 100.0, current_width, next_width);
            sep.change_strip_width(next_width, None);
            
            // Force the fixed height to match the new width (Square constraint)
            sep.prob.instance.base_strip.fixed_height = next_width;
	    // Apply the shrink to the variable dimension
	    sep.change_strip_width(next_width, None);

            current_width = next_width;
            infeas_sol_pool.clear();
        } else {
            info!("[EXPL] unable to reach feasibility (width: {:.3}, dens: {:.3}%, min loss: {:.3})", current_width, sep.prob.density() * 100.0, FMT().fmt2(total_loss));
            sol_listener.report(ReportType::ExplInfeas, &local_best.0, instance);

            // Separation was not successful add it to the pool of infeasible solutions
            match infeas_sol_pool.binary_search_by(|(_, o)| o.partial_cmp(&total_loss).unwrap()) {
                Ok(idx) | Err(idx) => infeas_sol_pool.insert(idx, (local_best.0.clone(), total_loss)),
            }

            if solution_pool.len() >= config.max_conseq_failed_attempts.unwrap_or(usize::MAX) {
	    	// === CHANGE START ===
                // Logic to recover from over-shrinking by increasing square size slightly
                if ENABLE_ADAPTIVE_SQUARE_RECOVERY {
                    // Back off by half the shrink step (e.g., if we shrank by 10%, grow by 5%)
                    let backoff_ratio = config.shrink_step * 0.5;
                    let next_width = current_width * (1.0 + backoff_ratio);
                    
                    info!("[EXPL] max consecutive failed attempts ({}) reached. ADAPTIVE: Backing off square size {:.3} -> {:.3}", solution_pool.len(), current_width, next_width);

                    // Update square dimensions
                    sep.prob.instance.base_strip.fixed_height = next_width;
                    sep.change_strip_width(next_width, None);
                    current_width = next_width;

                    // Reset the pool to restart attempts at this new, slightly easier size
                    solution_pool.clear();
                    
                    // Skip the disruption logic below and immediately try to separate at the new size
                    continue; 
                } else {
                    info!("[EXPL] max consecutive failed attempts ({}), terminating", solution_pool.len());
                    break;
                }
                // === CHANGE END ===            
            }

            // Restore to a random solution from the pool, with better solutions having more chance to be selected
            let selected_sol = {
                // Sample a value in range [0.0, 1.0[ from a normal distribution
                let distribution = Normal::new(0.0, config.solution_pool_distribution_stddev).unwrap();
                let sample = distribution.sample(&mut sep.rng).abs().min(0.999);
                // Map it to an index in the infeasible solution pool (better solutions are at the start of the pool)
                let selected_idx = (sample * infeas_sol_pool.len() as f32) as usize;

                let (selected_sol, loss) = &infeas_sol_pool[selected_idx];
                info!("[EXPL] starting solution {}/{} selected from solution pool (l: {}) to disrupt", selected_idx, infeas_sol_pool.len(), FMT().fmt2(*loss));
                selected_sol
            };

            // Rollback to this solution and disrupt it.
            sep.rollback(selected_sol, None);
            disrupt_solution(sep, config);
        }
    }

    info!("[EXPL] finished, best feasible solution: width: {:.3} ({:.3}%)",best_width,feasible_sols.last().unwrap().density(instance) * 100.0);

    feasible_sols
}


fn disrupt_solution(sep: &mut Separator, config: &ExplorationConfig) {

    let movable_items_count = sep.prob.layout.placed_items.iter().filter(|(_, pi)| !pi.is_locked).count();
    
    if movable_items_count < 2 {
        warn!("[DSRP] cannot disrupt solution with less than 2 movable items");
        return;
    }
    if sep.prob.layout.placed_items.len() < 2 {
        warn!("[DSRP] cannot disrupt solution with less than 2 items");
        return;
    }

    // The general idea is to disrupt a solution by swapping two 'large' items in the layout.
    // 'Large' items are those whose convex hull area falls within a certain top percentile
    // of the total convex hull area of all items in the layout.

    // Step 1: Define what constitutes a 'large' item.

    // Calculate the total convex hull area of all items, considering quantities.
    let total_convex_hull_area: f32 = sep
        .prob
        .instance
        .items
        .iter()
        .map(|(item, quantity)| item.shape_cd.surrogate().convex_hull_area * (*quantity as f32))
        .sum();

    let cutoff_threshold_area = total_convex_hull_area * config.large_item_ch_area_cutoff_percentile;

    // Sort items by convex hull area in descending order.
    let sorted_items_by_ch_area = sep
        .prob
        .instance
        .items
        .iter()
        .sorted_by_key(|(item, _)| Reverse(OrderedFloat(item.shape_cd.surrogate().convex_hull_area)))
        .peekable();

    let mut cumulative_ch_area = 0.0;
    let mut ch_area_cutoff = 0.0;

    // Iterate through items, accumulating their convex hull areas until the cumulative sum
    // exceeds the cutoff_threshold_area. The convex hull area of the item that causes
    // this excess becomes the ch_area_cutoff.
    for (item, quantity) in sorted_items_by_ch_area {
        let item_ch_area = item.shape_cd.surrogate().convex_hull_area;
        cumulative_ch_area += item_ch_area * (*quantity as f32);
        if cumulative_ch_area > cutoff_threshold_area {
            ch_area_cutoff = item_ch_area;
            debug!("[DSRP] cutoff ch area: {}, for item id: {}, bbox: {:?}",ch_area_cutoff, item.id, item.shape_cd.bbox);
            break;
        }
    }

    // Step 2: Select two 'large' items and 'swap' them.

    let large_items = sep.prob.layout.placed_items.iter()
        .filter(|(_, pi)| !pi.is_locked) // <--- CRITICAL CHANGE
        .filter(|(_, pi)| pi.shape.surrogate().convex_hull_area >= ch_area_cutoff);

    //Choose a first item with a large enough convex hull
    let (pk1, pi1) = large_items.clone().choose(&mut sep.rng).expect("[DSRP] failed to choose first item");

    //Choose a second item with a large enough convex hull and different enough from the first.
    //If no such item is found, choose a random one.
    let (pk2, pi2) = large_items.clone()
        .filter(|(_, pi)|
            // Ensure the second item is different from the first
            !approx_eq!(f32, pi.shape.area,pi1.shape.area, epsilon = pi1.shape.area * 0.01) &&
                !approx_eq!(f32, pi.shape.diameter, pi1.shape.diameter, epsilon = pi1.shape.diameter * 0.01)
        )
        .choose(&mut sep.rng)
        .or_else(|| {
            sep.prob.layout.placed_items.iter()
                .filter(|(pk, _)| *pk != pk1) // Ensure the second item is not the same as the first
                .choose(&mut sep.rng)
        }) // As a fallback, choose any item
        .expect("[EXPL] failed to choose second item for disruption");

    // Step 3: Swap the two items' positions in the layout.

    let dt1_old = pi1.d_transf;
    let dt2_old = pi2.d_transf;

    // Make sure the swaps do not violate feasibility (rotation).
    let dt1_new = convert_sample_to_closest_feasible(dt2_old, sep.prob.instance.item(pi1.item_id));
    let dt2_new = convert_sample_to_closest_feasible(dt1_old, sep.prob.instance.item(pi2.item_id));

    info!("[EXPL] disrupting by swapping two large items (id: {} <-> {})", pi1.item_id, pi2.item_id);

    let pk1 = sep.move_item(pk1, dt1_new);
    let pk2 = sep.move_item(pk2, dt2_new);


    // Step 4: Move all items that are practically contained by one of the swapped items to the "empty space" created by the moved item.
    //         This is particularly important when huge items are swapped with smaller items. 
    //         The huge item will create a large empty space and many of the items which previously 
    //         surrounded the smaller one will be contained by the huge one.
    {
        // transformation to convert the contained items' position (relative to the old and new positions of the swapped items)
let converting_transformation = dt1_new.compose().inverse()
            .transform(&dt1_old.compose());

        // [FIX] Collect keys into a Vec to release the borrow on 'sep'
        let items_to_move: Vec<PItemKey> = practically_contained_items(&sep.prob.layout, pk1)
            .into_iter()
            .filter(|c1_pk| *c1_pk != pk2)
            // Use the lock check here
            .filter(|c1_pk| !sep.prob.layout.placed_items[*c1_pk].is_locked)
            .collect();

        for c1_pk in items_to_move {
            let c1_pi = &sep.prob.layout.placed_items[c1_pk];

            let new_dt = c1_pi.d_transf
                .compose()
                .transform(&converting_transformation)
                .decompose();

            let new_feasible_dt = convert_sample_to_closest_feasible(new_dt, sep.prob.instance.item(c1_pi.item_id));
            sep.move_item(c1_pk, new_feasible_dt);
        }
    }

    // Do the same for the second item
    {
        let converting_transformation = dt2_new.compose().inverse()
            .transform(&dt2_old.compose());

        // [FIX] Collect keys into a Vec here too
        let items_to_move: Vec<PItemKey> = practically_contained_items(&sep.prob.layout, pk2)
            .into_iter()
            .filter(|c2_pk| *c2_pk != pk1)
            .filter(|c2_pk| !sep.prob.layout.placed_items[*c2_pk].is_locked)
            .collect();

        for c2_pk in items_to_move {
            let c2_pi = &sep.prob.layout.placed_items[c2_pk];
            
            let new_dt = c2_pi.d_transf
                .compose()
                .transform(&converting_transformation)
                .decompose();

            let new_feasible_dt = convert_sample_to_closest_feasible(new_dt, sep.prob.instance.item(c2_pi.item_id));
            sep.move_item(c2_pk, new_feasible_dt);
        }
    }
}

/// Collects all items which point of inaccessibility (POI) is contained by pk_c's shape.
fn practically_contained_items(layout: &Layout, pk_c: PItemKey) -> Vec<PItemKey> {
    let pi_c = &layout.placed_items[pk_c];
    // Detect all collisions with the item pk_c's shape.
    let mut collector = SecondaryMap::new();
    layout.cde().collect_poly_collisions(&pi_c.shape, &mut collector);

    // Filter out the items that have their POI contained by pk_c's shape.
    collector.iter()
        .filter_map(|(_,he)| {
            match he {
                HazardEntity::PlacedItem { pk, .. } => Some(*pk),
                _ => None
            }
        })
        .filter(|pk| *pk != pk_c) // Ensure we don't include the item itself
        .filter(|pk| {
            // Check if the POI of the item is contained by pk_c's shape
            let poi = layout.placed_items[*pk].shape.poi;
            pi_c.shape.collides_with(&poi.center)
        })
        .collect_vec()
}
