mod navigation;
mod traveler;

use bevy_app::{App, Plugin};
use bevy_ecs::schedule::IntoSystemDescriptor;

pub use navigation::{NavGraph, NavPoint, NavPointRef};
use traveler::{compute_initial_path, move_travelers};
pub use traveler::{AutoTraveler, TravelingPaused};

#[derive(Default, Clone, Copy)]
pub struct NavigatorPlugin {
    pub initial_graph_capacity: usize,
}

impl NavigatorPlugin {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(mut self, capacity: usize) -> Self {
        self.initial_graph_capacity = capacity;
        self
    }
}

impl Plugin for NavigatorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(NavGraph::with_capacity(self.initial_graph_capacity))
            .add_system(compute_initial_path.label("compute_path"))
            .add_system(move_travelers.after("compute_path"))
            .register_type::<AutoTraveler>()
            .register_type::<NavPointRef>();
    }
}
