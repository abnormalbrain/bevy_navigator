use std::{
    cmp::{Ordering, Reverse},
    collections::{BinaryHeap, VecDeque},
};

use bevy_math::Vec3;
use bevy_reflect::prelude::*;
use bevy_utils::{HashMap, HashSet};

#[derive(Debug, Reflect, FromReflect)]
pub struct NavPoint {
    id: u32,
    location: Vec3,
    speed_modifier: f32,
    connections: HashSet<u32>,
    max_occupancy: u32,
    current_occupancy: u32,
}

impl NavPoint {
    pub fn new(id: u32, location: Vec3, speed_modifier: f32, max_occupancy: u32) -> Self {
        Self {
            id,
            location,
            speed_modifier,
            connections: HashSet::new(),
            max_occupancy,
            current_occupancy: 0,
        }
    }

    #[inline(always)]
    pub fn id(&self) -> u32 {
        self.id
    }

    #[inline(always)]
    pub fn location(&self) -> Vec3 {
        self.location
    }

    #[inline(always)]
    pub fn speed_modifier(&self) -> f32 {
        self.speed_modifier
    }

    #[inline(always)]
    pub fn max_occupancy(&self) -> u32 {
        self.max_occupancy
    }

    #[inline(always)]
    pub fn current_occupancy(&self) -> u32 {
        self.current_occupancy
    }

    #[inline(always)]
    pub fn can_occupy(&self) -> bool {
        self.current_occupancy < self.max_occupancy
    }

    #[inline(always)]
    pub fn occupy(&mut self) -> bool {
        if self.can_occupy() {
            self.current_occupancy += 1;
            true
        } else {
            false
        }
    }

    pub fn unoccupy(&mut self) {
        self.current_occupancy = (self.current_occupancy - 1).max(0);
    }
}

pub(crate) struct NavPointIdCounter(u32);

impl Default for NavPointIdCounter {
    fn default() -> Self {
        Self(1)
    }
}

pub(crate) struct NavPointIdFreelist(VecDeque<u32>);

impl NavPointIdFreelist {
    pub fn new() -> Self {
        Self(VecDeque::with_capacity(500))
    }

    pub fn freed(&mut self, id: u32) {
        self.0.push_back(id);
    }

    pub fn next(&mut self) -> Option<u32> {
        self.0.pop_front()
    }
}

#[derive(Debug, Default)]
pub struct NavGraph {
    points: HashMap<u32, NavPoint>,
    highest_id: u32,
}

#[derive(Eq)]
struct PathNode {
    id: u32,
    f: u32,
}

impl PartialEq for PathNode {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl PartialOrd for PathNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.f.cmp(&other.f))
    }
}

impl Ord for PathNode {
    fn cmp(&self, other: &Self) -> Ordering {
        self.f.cmp(&other.f)
    }
}

impl NavGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn add_nav_point(&mut self, point: NavPoint) {
        for connection in &point.connections {
            self.points.entry(*connection).and_modify(|b| {
                b.connections.insert(point.id);
            });
        }

        if point.id > self.highest_id {
            self.highest_id = point.id;
        }
        self.points.insert(point.id, point);
    }

    pub fn connect_points(&mut self, a: u32, b: u32) {
        if !self.has_node(a) || !self.has_node(b) {
            return;
        }

        self.points.entry(a).and_modify(|point| {
            point.connections.insert(b);
        });
        self.points.entry(b).and_modify(|point| {
            point.connections.insert(a);
        });
    }

    #[inline(always)]
    pub fn has_node(&self, id: u32) -> bool {
        self.points.contains_key(&id)
    }

    pub fn remove_point(&mut self, id: u32) {
        if let Some(point) = self.points.remove(&id) {
            for connection in &point.connections {
                self.points.entry(*connection).and_modify(|b| {
                    b.connections.remove(&point.id);
                });
            }
        }
    }

    pub fn can_occupy(&self, id: u32) -> bool {
        self.points
            .get(&id)
            .map(|p| p.can_occupy())
            .unwrap_or(false)
    }

    pub fn occupy(&mut self, id: u32) -> bool {
        let mut occupied = false;
        self.points.entry(id).and_modify(|p| {
            occupied = p.occupy();
        });
        occupied
    }

    pub fn unoccupy(&mut self, id: u32) {
        self.points.entry(id).and_modify(|p| {
            p.unoccupy();
        });
    }

    #[inline(always)]
    fn h_func(&self, a: &u32, b: &u32) -> u32 {
        if let (Some(a_node), Some(b_node)) = (self.points.get(a), self.points.get(b)) {
            (a_node.location.distance_squared(b_node.location) / b_node.speed_modifier * 100.0)
                as u32
        } else {
            u32::MAX
        }
    }

    pub fn find_path(&self, a: u32, b: u32) -> Option<Vec<u32>> {
        let mut cap_guess = 0_usize;
        if let (Some(a_node), Some(b_node)) = (self.points.get(&a), self.points.get(&b)) {
            // Straight line dist * 2 as a general estimate.
            // This may over-allocate in some scenarios but accounts for a 15-20% reduction
            // in computation time to keep from having to resize all of the collections frequently.
            cap_guess = (a_node.location().distance(b_node.location()) * 2.0) as usize;
        } else {
            return None;
        }

        let mut search_ids = HashSet::<u32>::with_capacity(cap_guess);
        let mut open_set = BinaryHeap::with_capacity(cap_guess);
        let mut came_from = HashMap::<u32, u32>::with_capacity(cap_guess);
        let mut g_score = HashMap::<u32, u32>::with_capacity(cap_guess);
        let mut f_score = HashMap::<u32, u32>::with_capacity(cap_guess);

        let start_h = self.h_func(&a, &b);
        let start_node = PathNode { id: a, f: start_h };
        g_score.insert(a, 0);
        f_score.insert(a, start_node.f);
        search_ids.insert(start_node.id);
        open_set.push(Reverse(start_node));

        while let Some(Reverse(current)) = open_set.pop() {
            if current.id == b {
                let mut total_path = VecDeque::with_capacity(cap_guess);
                let mut prev = current.id;
                while prev != a {
                    total_path.push_front(prev);
                    prev = came_from[&prev];
                }
                return Some(total_path.into());
            }

            search_ids.remove(&current.id);

            if !self.points.contains_key(&current.id) {
                continue;
            }

            for neighbor_id in &self.points[&current.id].connections {
                let neighbor = &self.points[neighbor_id];
                if !neighbor.can_occupy() {
                    continue;
                }
                let tentative_g_score =
                    g_score[&current.id] + self.h_func(&current.id, &neighbor.id);
                if tentative_g_score < *g_score.entry(*neighbor_id).or_insert(u32::MAX) {
                    came_from.insert(*neighbor_id, current.id);
                    let cur_h_score = self.h_func(neighbor_id, &b);
                    let cur_f_score = tentative_g_score + cur_h_score;

                    g_score.insert(*neighbor_id, tentative_g_score);
                    f_score.insert(*neighbor_id, cur_f_score);

                    if !search_ids.contains(neighbor_id) {
                        search_ids.insert(*neighbor_id);
                        open_set.push(Reverse(PathNode {
                            id: *neighbor_id,
                            f: cur_f_score,
                        }));
                    }
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    pub fn test_basic_route() {
        let mut nav_graph = NavGraph::new();
        nav_graph.add_nav_point(NavPoint::new(1, Vec3::new(0.0, 0.0, 0.0), 1.0, 1));
        nav_graph.add_nav_point(NavPoint::new(2, Vec3::new(0.0, 1.0, 0.0), 1.0, 1));
        nav_graph.add_nav_point(NavPoint::new(3, Vec3::new(0.0, 2.0, 0.0), 1.0, 1));

        nav_graph.connect_points(1, 2);
        nav_graph.connect_points(2, 3);

        let path = nav_graph.find_path(1, 3);
        assert!(path.is_some());
        let p = path.unwrap();
        assert_eq!(p.len(), 2);
        assert_eq!(p[0], 2);
        assert_eq!(p[1], 3);
    }

    #[test]
    pub fn test_occupancy() {
        let mut nav_graph = NavGraph::new();
        nav_graph.add_nav_point(NavPoint::new(1, Vec3::new(0.0, 0.0, 0.0), 1.0, 1));
        nav_graph.add_nav_point(NavPoint::new(2, Vec3::new(0.0, 1.0, 0.0), 1.0, 1));
        nav_graph.add_nav_point(NavPoint::new(3, Vec3::new(1.0, 1.0, 0.0), 1.0, 1));
        nav_graph.add_nav_point(NavPoint::new(4, Vec3::new(0.0, 2.0, 0.0), 1.0, 1));

        nav_graph.connect_points(1, 2);
        nav_graph.connect_points(1, 3);
        nav_graph.connect_points(2, 4);
        nav_graph.connect_points(3, 4);
        nav_graph.occupy(2);

        let path = nav_graph.find_path(1, 4).unwrap();
        assert_eq!(path[0], 3);
        assert_eq!(path[1], 4);

        nav_graph.occupy(3);
        assert!(nav_graph.find_path(1, 4).is_none());

        nav_graph.unoccupy(2);
        assert_eq!(nav_graph.find_path(1, 4).unwrap()[0], 2);
    }
}
