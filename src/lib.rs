mod navigation;

use bevy_app::{App, Plugin};
pub use navigation::{NavGraph, NavPoint};

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
        app.insert_resource(NavGraph::with_capacity(self.initial_graph_capacity));
    }
}
