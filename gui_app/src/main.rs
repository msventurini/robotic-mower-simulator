// Main.rs - Entry point for the lawn mowing robot simulation
// This file sets up the simulation parameters and controls the main simulation loop
use std::time::Instant;

use simulation_worker::{GrassMap, PathMode, Robot, simulation, Vector2};
// use gui_app::{GrassMap, PathMode, Robot, RobotState, Vector2};



fn main() {
    // Create a grass map from an image file
    // - Cell size: 0.01m (1cm)
    // - Simulation ticks: 10 per second
    // - Growth rate: 8mm per day (8.0 / 24.0 / 60.0 / 60.0 mm per second)
    let mut grassmap = GrassMap::new_from_image(
        //
        "./Maps_PNG/IA01BT1009_Map.png",
        0.01, // 1cm cell size
        10,   // 10 ticks per second
        0.0,  //8.0 / 24.0 / 60.0 / 60.0, // 8mm growth per day
    );

    // Create a new robot with the following parameters:
    let robot = Robot::new(
        &mut grassmap,
        0.01,             // 1cm cell size
        Vector2::ZERO,    // Starting position MUST be at (0,0)
        Vector2::ZERO,    // Home position MUST be at (0,0)
        0.5,              // Speed: 0.5 meters per second
        0.14,             // Cutting radius: 14cm
        30.0,             // Target cut height: 30mm
        80.0 / 3600.0,    // Battery drain: 80% per 1 hours of movement
        0.0,              // No battery drain from cutting (simplified model)
        PathMode::Random, // Move in Random
        2.0,              // Zigzag width: 2 meters
    );

    // Run simulation for 1 day (in seconds), saving every 600 seconds (10 minutes)
    simulation::run(robot, 1 * 24 * 3600, 600);
}
