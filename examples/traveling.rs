use bevy::{
    prelude::{App, AssetServer, Camera2dBundle, Vec3},
    sprite::SpriteBundle,
    DefaultPlugins,
};
use bevy_ecs::system::{Commands, Res, ResMut};
use bevy_navigator::{AutoTraveler, NavGraph, NavPoint, NavPointRef, NavigatorPlugin};
use bevy_transform::prelude::Transform;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(NavigatorPlugin::new())
        .add_startup_system(setup)
        .run();
}

fn setup(mut nav_graph: ResMut<NavGraph>, asset_server: Res<AssetServer>, mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
    let mut id = 1;
    for y in -20..20 {
        for x in -20..20 {
            let location = Vec3::new(x as f32 * 16.0, y as f32 * 16.0, 1.0);
            commands
                .spawn(SpriteBundle {
                    texture: asset_server.load("tile.png"),
                    transform: Transform::from_xyz(location.x, location.y, location.z - 1.0),
                    ..Default::default()
                })
                .insert(NavPointRef(id));
            nav_graph.add_nav_point(NavPoint::new(id, location, 1.0, 1));
            if id > 40 {
                nav_graph.connect_points(id, id - 40);
            }
            if x != -20 && id > 1 {
                nav_graph.connect_points(id, id - 1);
            }
            id += 1;
        }
    }

    commands
        .spawn(SpriteBundle {
            texture: asset_server.load("ball.png"),
            transform: Transform::from_xyz(-20.0 * 16.0, -20.0 * 16.0, 1.0),
            ..Default::default()
        })
        .insert(AutoTraveler::new(1, id - 1, 100.0));
    nav_graph.occupy(1);
}
