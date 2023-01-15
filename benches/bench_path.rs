use bevy_math::Vec3;
use criterion::*;

use bevy_navigator::{NavGraph, NavPoint};

fn create_grid(size: u32, dense: bool) -> NavGraph {
    let mut nav_graph = NavGraph::new();
    let mut id = 1_u32;
    for x in 1..=size {
        for y in 1..=size {
            nav_graph.add_nav_point(NavPoint::new(
                id,
                Vec3::new(x as f32, y as f32, 0.0),
                1.0,
                1,
            ));
            nav_graph.connect_points(id, id - 1);
            nav_graph.connect_points(id, id - size);
            if dense {
                nav_graph.connect_points(id, id - size - 1);
                nav_graph.connect_points(id, id - size + 1);
            }
            id += 1;
        }
    }

    nav_graph
}

fn bench_path(c: &mut Criterion) {
    let small_sparse = create_grid(10, false);
    let small_dense = create_grid(10, true);

    let medium_sparse = create_grid(500, false);
    let medium_dense = create_grid(500, true);

    let large_sparse = create_grid(1000, false);
    let large_dense = create_grid(1000, true);

    let mut small_sparse_group = c.benchmark_group("Small Sparse Nav");
    small_sparse_group.measurement_time(core::time::Duration::from_secs(30));

    small_sparse_group.bench_function("long", |b| b.iter(|| small_sparse.find_path(1, 100)));
    small_sparse_group.bench_function("short", |b| b.iter(|| small_sparse.find_path(25, 45)));

    small_sparse_group.finish();

    let mut small_dense_group = c.benchmark_group("Small Dense Nav");
    small_dense_group.measurement_time(core::time::Duration::from_secs(30));

    small_dense_group.bench_function("long", |b| b.iter(|| small_dense.find_path(1, 100)));
    small_dense_group.bench_function("short", |b| b.iter(|| small_dense.find_path(25, 45)));

    small_dense_group.finish();

    let mut medium_sparse_group = c.benchmark_group("Medium Sparse Nav");
    medium_sparse_group.measurement_time(core::time::Duration::from_secs(30));

    medium_sparse_group.bench_function("long", |b| b.iter(|| medium_sparse.find_path(1, 250000)));
    medium_sparse_group.bench_function("short", |b| b.iter(|| medium_sparse.find_path(250, 1250)));

    medium_sparse_group.finish();

    let mut medium_dense_group = c.benchmark_group("Medium Dense Nav");
    medium_dense_group.measurement_time(core::time::Duration::from_secs(30));

    medium_dense_group.bench_function("long", |b| b.iter(|| medium_dense.find_path(1, 250000)));
    medium_dense_group.bench_function("short", |b| b.iter(|| medium_dense.find_path(250, 1250)));

    medium_dense_group.finish();

    let mut large_sparse_group = c.benchmark_group("Large Sparse Nav");
    large_sparse_group.measurement_time(core::time::Duration::from_secs(60));

    large_sparse_group.bench_function("long", |b| b.iter(|| large_sparse.find_path(1, 1000000)));
    large_sparse_group.bench_function("short", |b| b.iter(|| large_sparse.find_path(500, 2500)));

    large_sparse_group.finish();

    let mut large_dense_group = c.benchmark_group("Large Dense Nav");
    large_dense_group.measurement_time(core::time::Duration::from_secs(60));

    large_dense_group.bench_function("long", |b| b.iter(|| large_dense.find_path(1, 1000000)));
    large_dense_group.bench_function("short", |b| b.iter(|| large_dense.find_path(500, 2500)));

    large_dense_group.finish();
}

criterion_group!(benches, bench_path);
criterion_main!(benches);
