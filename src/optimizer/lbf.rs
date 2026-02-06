use crate::eval::lbf_evaluator::LBFEvaluator;
use crate::eval::sample_eval::SampleEval;
use crate::sample::search::{search_placement, SampleConfig};
use itertools::Itertools;
use log::debug;
use log::warn;
use log::info;
use ordered_float::OrderedFloat;
use std::cmp::Reverse;
use std::iter;
use jagua_rs::Instant;
use jagua_rs::entities::Instance;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use rand_xoshiro::Xoshiro256PlusPlus;
use crate::util::assertions;

pub struct LBFBuilder {
    pub instance: SPInstance,
    pub prob: SPProblem,
    pub rng: Xoshiro256PlusPlus,
    pub sample_config: SampleConfig,
}

impl LBFBuilder {
    pub fn new(
        instance: SPInstance,
        rng: Xoshiro256PlusPlus,
        sample_config: SampleConfig,
    ) -> Self {
        let prob = SPProblem::new(instance.clone());

        Self {
            instance,
            prob,
            rng,
            sample_config,
        }
    }

    pub fn construct(mut self) -> Self {
        let start = Instant::now();
        // [CHANGE] Enforce fixed items (Unmelted Crystal) before greedy construction
        self.enforce_fixed_items();
        
        let n_items = self.instance.items.len();
        let sorted_item_indices = (0..n_items)
            .sorted_by_cached_key(|id| {
                let item_shape = self.instance.item(*id).shape_cd.as_ref();
                let convex_hull_area = item_shape.surrogate().convex_hull_area;
                let diameter = item_shape.diameter;
                Reverse(OrderedFloat(convex_hull_area * diameter))
            })
            .flat_map(|id| {
                let missing_qty = self.prob.item_demand_qtys[id];
                iter::repeat_n(id, missing_qty)
            })
            .collect_vec();

        info!("[CONSTR] placing items in order: {:?}",sorted_item_indices);

        for item_id in sorted_item_indices {
            self.place_item(item_id);
        }

        self.prob.fit_strip();
        info!("[CONSTR] placed all items in width: {:.3} (in {:?})",self.prob.strip_width(), start.elapsed());
        self
    }

    fn place_item(&mut self, item_id: usize) {
        match self.find_placement(item_id) {
            Some(p_opt) => {
                self.prob.place_item(p_opt);
                info!("[CONSTR] placing item {}/{} with id {} at [{}]",self.prob.layout.placed_items.len(),self.instance.total_item_qty(),p_opt.item_id,p_opt.d_transf);
            }
            None => {
                info!("[CONSTR] failed to place item with id {}, expanding strip width",item_id);
                self.prob.change_strip_width(self.prob.strip_width() * 1.2);
                assert!(assertions::strip_width_is_in_check(&self.prob), "strip-width is running away (>{:.3}), item {item_id} does not seem to fit into the strip", self.prob.strip_width());          
                self.place_item(item_id);
            }
        }
    }

    fn find_placement(&mut self, item_id: usize) -> Option<SPPlacement> {
	let layout = &self.prob.layout;
	let item = self.instance.item(item_id);
	    
	// [CHANGE] Create a factory closure instead of the instance directly
	let evaluator_factory = || LBFEvaluator::new(layout, item);

	// Pass the factory to search_placement
	let (best_sample, _) = search_placement(layout, item, None, evaluator_factory, self.sample_config, &mut self.rng);

	match best_sample {
	   Some((d_transf, SampleEval::Clear { .. })) => {
	      Some(SPPlacement { item_id, d_transf })
	   }
	   _ => None
	}
    }
    
    // [CHANGE] Logic moved from explore.rs
    fn enforce_fixed_items(&mut self) {
        // Collect items to fix first to avoid borrow conflicts
        let items_to_fix = self.instance.items()
            .filter_map(|item| item.fixed_placement.map(|fp| (item.id, fp)))
            .collect::<Vec<_>>();

        let mut fixed_count = 0;

        for (item_id, d_transf) in items_to_fix {
            // Only place if there is demand (safeguard)
            if self.prob.item_demand_qtys[item_id] > 0 {
                let placement = SPPlacement { item_id, d_transf };
                
                // Place the item (this handles demand decrement and CDE registration)
                // Note: Standard placement is unlocked (false).
                self.prob.place_item(placement);
                info!("[CONSTR] placing item {}/{} with id {} at [{}]",self.prob.layout.placed_items.len(),self.instance.total_item_qty(),item_id,d_transf);

                // Immediately find and lock the item we just placed
                // We assume it's one of the items with this ID that isn't locked yet.
                if let Some((_, pi)) = self.prob.layout.placed_items.iter_mut()
                    .find(|(_, pi)| pi.item_id == item_id && !pi.is_locked) 
                {
                    pi.is_locked = true;
                    fixed_count += 1;
                } else {
                     warn!("[CONSTR] Failed to lock fixed item {}", item_id);
                }
            } else {
                 warn!("[CONSTR] Item {} has fixed placement but no demand left.", item_id);
            }
        }

        if fixed_count > 0 {
            info!("[CONSTR] Enforced {} fixed items (unmelted crystal)", fixed_count);
        }
    }
}
