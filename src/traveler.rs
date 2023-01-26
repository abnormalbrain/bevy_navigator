use bevy_ecs::{
    component::Component,
    entity::Entity,
    query::{Added, Without},
    system::{Commands, Query, Res, ResMut},
};
use bevy_reflect::{FromReflect, Reflect};
use bevy_time::Time;
use bevy_transform::prelude::Transform;
use bevy_utils::tracing::info;

use crate::NavGraph;

#[derive(Debug, Reflect, FromReflect, Clone, Copy)]
pub enum BlockedBehavior {
    Wait,
    Recompute,
}

impl Default for BlockedBehavior {
    fn default() -> Self {
        Self::Recompute
    }
}

#[derive(Debug, Reflect, FromReflect, Clone, Copy)]
pub enum DestinationBehavior {
    Exactly,
    WithinRadius(f32),
}

impl Default for DestinationBehavior {
    fn default() -> Self {
        Self::Exactly
    }
}

#[derive(Debug, Reflect, FromReflect, Clone, Copy)]
pub enum PathBehavior {
    Precompute,
    ProgressiveRecompute,
}

impl Default for PathBehavior {
    fn default() -> Self {
        Self::Precompute
    }
}

#[derive(Debug, Reflect, FromReflect, Component, Clone)]
pub struct AutoTraveler {
    pub origin: u32,
    pub destination: u32,
    pub path: Option<Vec<u32>>,
    pub current_index: usize,
    pub speed: f32,
    pub blocked_behavior: BlockedBehavior,
    pub destination_behavior: DestinationBehavior,
    pub path_behavior: PathBehavior,
}

impl Default for AutoTraveler {
    fn default() -> Self {
        Self {
            origin: 0,
            destination: 0,
            path: None,
            current_index: 0,
            speed: 1.0,
            blocked_behavior: BlockedBehavior::default(),
            destination_behavior: DestinationBehavior::default(),
            path_behavior: PathBehavior::default(),
        }
    }
}

impl AutoTraveler {
    pub fn new(origin: u32, destination: u32, speed: f32) -> Self {
        Self {
            origin,
            destination,
            speed,
            ..Default::default()
        }
    }

    pub fn with_blocked_behavior(mut self, blocked_behavior: BlockedBehavior) -> Self {
        self.blocked_behavior = blocked_behavior;
        self
    }

    pub fn with_destination_behavior(mut self, destination_behavior: DestinationBehavior) -> Self {
        self.destination_behavior = destination_behavior;
        self
    }

    pub fn with_path_behavior(mut self, path_behavior: PathBehavior) -> Self {
        self.path_behavior = path_behavior;
        self
    }
}

#[derive(Debug, Component, Reflect, FromReflect)]
pub struct NoPath;

#[derive(Debug, Component, Reflect, FromReflect)]
pub struct TravelingPaused;

#[derive(Debug, Component, Reflect, FromReflect)]
pub struct TravelerPosition {
    pub current_nav_point: u32,
    pub next_nav_point: Option<u32>,
}

pub(crate) fn compute_initial_path(
    mut new_travelers_query: Query<(Entity, &mut AutoTraveler), Added<AutoTraveler>>,
    nav_graph: Res<NavGraph>,
    mut commands: Commands,
) {
    for (entity, mut auto_traveler) in new_travelers_query.iter_mut() {
        if let Some(path) = nav_graph.find_path(auto_traveler.origin, auto_traveler.destination) {
            commands.entity(entity).insert(TravelerPosition {
                current_nav_point: auto_traveler.origin,
                next_nav_point: None,
            });
            info!("Found path: {:?}", &path);
            auto_traveler.path = Some(path);
        } else {
            info!("No path found");
            commands.entity(entity).insert(NoPath);
        }
    }
}

pub(crate) fn move_travelers(
    mut moving_travelers_query: Query<
        (
            Entity,
            &mut Transform,
            &mut AutoTraveler,
            &mut TravelerPosition,
        ),
        Without<TravelingPaused>,
    >,
    mut nav_graph: ResMut<NavGraph>,
    time: Res<Time>,
    mut commands: Commands,
) {
    for (entity, mut transform, mut auto_traveler, mut traveler_position) in
        moving_travelers_query.iter_mut()
    {
        let mut should_advance = false;
        if let Some(path) = auto_traveler.path.as_ref() {
            if auto_traveler.current_index + 1 >= path.len() {
                commands.entity(entity).remove::<AutoTraveler>();
                continue;
            }

            if traveler_position.next_nav_point.is_none() {
                if nav_graph.occupy(path[auto_traveler.current_index + 1]) {
                    traveler_position.next_nav_point = Some(path[auto_traveler.current_index + 1]);
                } else {
                    // determine based on BlockedBehavior
                    info!("Travel blocked");
                    continue;
                }
            }

            if let (Some(from), Some(to)) = (
                nav_graph.get_nav_point(traveler_position.current_nav_point),
                nav_graph.get_nav_point(traveler_position.next_nav_point.unwrap()),
            ) {
                let direction = (to.location() - from.location()).normalize();
                let movement =
                    direction * auto_traveler.speed * from.speed_modifier() * time.delta_seconds();

                let movement_len_squared = movement.length_squared();
                let dist_squared = transform.translation.distance_squared(to.location());

                // Check if we're going to overshoot or are within the move threshold and just snap to the destination instead.
                if movement_len_squared >= dist_squared || dist_squared <= 0.001_f32.powi(2) {
                    transform.translation = to.location();
                    should_advance = true;
                    nav_graph.unoccupy(traveler_position.current_nav_point);
                    traveler_position.current_nav_point = path[auto_traveler.current_index + 1];
                    traveler_position.next_nav_point = None;
                } else {
                    transform.translation += movement;
                }
            }
        }

        if should_advance {
            auto_traveler.current_index += 1;
        }
    }
}
