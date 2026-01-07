use crate::optimizer::separator::SeparatorConfig;
use crate::sample::search::SampleConfig;
use jagua_rs::collision_detection::CDEConfig;
use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
use std::time::Duration;

#[derive(Debug, Clone, Copy)]
pub struct SparrowConfig {
    pub rng_seed: Option<usize>,
    pub expl_cfg: ExplorationConfig,
    pub cmpr_cfg: CompressionConfig,
    /// Configuration for the collision detection engine.
    /// See [`CDEConfig`] for more details.
    pub cde_config: CDEConfig,
    /// Defines the polygon simplification tolerance: maximum allowable inflation of items when simplifying their shape.
    /// Disabled if `None`.
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub poly_simpl_tolerance: Option<f32>,
    /// Defines the minimum distance between items and other hazards.
    /// Disabled if `None`.
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub min_item_separation: Option<f32>,
    /// Defines the maximum distance between two vertices of a polygon to consider it a narrow concavity (which will be closed).
    /// Disabled if `None`.
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub narrow_concavity_cutoff_ratio: Option<f32>,
}

#[derive(Debug, Clone, Copy)]
pub struct ExplorationConfig {
    pub shrink_step: f32,
    pub time_limit: Duration,
    pub max_conseq_failed_attempts: Option<usize>,
    pub solution_pool_distribution_stddev: f32,
    pub separator_config: SeparatorConfig,
    pub large_item_ch_area_cutoff_percentile: f32
}

#[derive(Debug, Clone, Copy)]
pub struct CompressionConfig {
    pub shrink_range: (f32, f32),
    pub time_limit: Duration,
    pub shrink_decay: ShrinkDecayStrategy,
    pub separator_config: SeparatorConfig,
}

#[derive(Debug, Clone, Copy)]
pub enum ShrinkDecayStrategy {
    /// The shrink ratio decays linearly with time
    TimeBased,
    /// The shrink ratio decays by a fixed ratio every time it fails to compress into a feasible solution
    FailureBased(f32),
}

pub const DEFAULT_SPARROW_CONFIG: SparrowConfig = SparrowConfig {
    rng_seed: None,
    expl_cfg: ExplorationConfig {
        shrink_step: 0.002,
        time_limit: Duration::from_secs(9 * 60),
        max_conseq_failed_attempts: Some(10),
        solution_pool_distribution_stddev: 0.05,
        separator_config: SeparatorConfig {
            iter_no_imprv_limit: 300,
            strike_limit: 6,
            log_level: log::Level::Info,
            n_workers: 6,
            sample_config: SampleConfig {
                n_container_samples: 300,
                n_focussed_samples: 5,
                n_coord_descents: 4,
            },
        },
        large_item_ch_area_cutoff_percentile: 0.90
    },
    cmpr_cfg: CompressionConfig {
        shrink_range: (0.0001, 0.00001),
        time_limit: Duration::from_secs(60),
        shrink_decay: ShrinkDecayStrategy::TimeBased,
        separator_config: SeparatorConfig {
            iter_no_imprv_limit: 100,
            strike_limit: 5,
            log_level: log::Level::Debug,
            n_workers: 4,
            sample_config: SampleConfig {
                n_container_samples: 500,
                n_focussed_samples: 15,
                n_coord_descents: 8,
            },
        },
    },
    cde_config: CDEConfig {
        quadtree_depth: 5,
        cd_threshold: 16,
        item_surrogate_config: SPSurrogateConfig {
            n_pole_limits: [(32, 0.0), (16, 1.5), (8, 22.5)],
            n_ff_poles: 1,
            n_ff_piers: 1,
        },
    },
    poly_simpl_tolerance: Some(0.0001),
    narrow_concavity_cutoff_ratio: Some(0.01),
    min_item_separation: None,
};
