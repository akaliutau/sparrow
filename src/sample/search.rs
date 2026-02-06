use jagua_rs::entities::{Item, Layout, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use crate::consts::{SND_REFINE_CD_TL_RATIOS, PRE_REFINE_CD_TL_RATIOS, UNIQUE_SAMPLE_THRESHOLD, PRE_REFINE_CD_R_STEPS, SND_REFINE_CD_R_STEPS};
use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use crate::sample::best_samples::BestSamples;
use crate::sample::coord_descent::{refine_coord_desc, CDConfig};
use crate::sample::uniform_sampler::UniformBBoxSampler;
use log::debug;
use rand::Rng;
use rayon::prelude::*; // Ensure rayon is imported

#[derive(Debug, Clone, Copy)]
pub struct SampleConfig {
    pub n_container_samples: usize,
    pub n_focussed_samples: usize,
    pub n_coord_descents: usize,
}

/// Algorithm 6: Parallelized Search
pub fn search_placement<E, F>(
    l: &Layout,
    item: &Item,
    ref_pk: Option<PItemKey>,
    evaluator_factory: F, // [CHANGE] Accept a Factory instead of an Instance
    sample_config: SampleConfig,
    rng: &mut impl Rng
) -> (Option<(DTransformation, SampleEval)>, usize) 
where 
    E: SampleEvaluator,
    F: Fn() -> E + Sync + Send, // Factory must be thread-safe
{
    let item_min_dim = f32::min(item.shape_cd.bbox.width(), item.shape_cd.bbox.height());
    let mut best_samples = BestSamples::new(sample_config.n_coord_descents, item_min_dim * UNIQUE_SAMPLE_THRESHOLD);

    // 1. Collect all samples first
    let mut samples = Vec::with_capacity(sample_config.n_focussed_samples + sample_config.n_container_samples + 1);

    // [Same sampling logic as before...]
    let focussed_sampler = match ref_pk {
        Some(ref_pk) => {
            samples.push(l.placed_items[ref_pk].d_transf);
            let pi_bbox = l.placed_items[ref_pk].shape.bbox;
            UniformBBoxSampler::new(pi_bbox, item, l.container.outer_cd.bbox)
        }
        None => None,
    };
    let _container_sampler = UniformBBoxSampler::new(l.container.outer_cd.bbox, item, l.container.outer_cd.bbox);

    //Perform the focussed sampling
    if let Some(focussed_sampler) = focussed_sampler {
        for _ in 0..sample_config.n_focussed_samples {
            samples.push(focussed_sampler.sample(rng));
        }
    }

    let container_sampler = UniformBBoxSampler::new(l.container.outer_cd.bbox, item, l.container.outer_cd.bbox);
    if let Some(container_sampler) = container_sampler {
        for _ in 0..sample_config.n_container_samples {
            samples.push(container_sampler.sample(rng));
        }
    }

    // 2. Parallel Evaluation
    // Use the factory to create a thread-local evaluator
    let evaluated_samples: Vec<(DTransformation, SampleEval)> = samples.par_iter()
        .map_init(
            &evaluator_factory, 
            |evaluator, &dt| {
                let eval = evaluator.evaluate_sample(dt, None); 
                (dt, eval)
            }
        )
        .collect();

    for (dt, eval) in evaluated_samples {
        best_samples.report(dt, eval);
    }
    
    // 3. Refinement (Sequential)
    // Create one local evaluator instance for the main thread
    let mut evaluator = evaluator_factory(); 

    // Prerefine
    for start in best_samples.samples.clone() {
        let descended = refine_coord_desc(start, &mut evaluator, prerefine_cd_config(item), rng);
        best_samples.report(descended.0, descended.1);
    }

    // Final refine
    let final_sample = best_samples.best().map(|s|
        refine_coord_desc(s, &mut evaluator, final_refine_cd_config(item), rng)
    );

    debug!("[S] {} samples evaluated, final: {:?}", samples.len() + evaluator.n_evals(), final_sample);
    (final_sample, samples.len() + evaluator.n_evals())
}

// ... [Keep helper config functions same as original]
fn prerefine_cd_config(item: &Item) -> CDConfig {
    // ... same content ...
    let item_min_dim = f32::min(item.shape_cd.bbox.width(), item.shape_cd.bbox.height());
    let wiggle = item.allowed_rotation == RotationRange::Continuous;
    CDConfig {
        t_step_init: item_min_dim * PRE_REFINE_CD_TL_RATIOS.0,
        t_step_limit: item_min_dim * PRE_REFINE_CD_TL_RATIOS.1,
        r_step_init: PRE_REFINE_CD_R_STEPS.0,
        r_step_limit: PRE_REFINE_CD_R_STEPS.1,
        wiggle,
    }
}

fn final_refine_cd_config(item: &Item) -> CDConfig {
    // ... same content ...
    let item_min_dim = f32::min(item.shape_cd.bbox.width(), item.shape_cd.bbox.height());
    let wiggle = item.allowed_rotation == RotationRange::Continuous;
    CDConfig {
        t_step_init: item_min_dim * SND_REFINE_CD_TL_RATIOS.0,
        t_step_limit: item_min_dim * SND_REFINE_CD_TL_RATIOS.1,
        r_step_init: SND_REFINE_CD_R_STEPS.0,
        r_step_limit: SND_REFINE_CD_R_STEPS.1,
        wiggle
    }
}
