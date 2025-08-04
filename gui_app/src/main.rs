use simulation_worker::{GrassMap, PathMode, Robot, simulation, Vector2};
// use gui_app::{GrassMap, PathMode, Robot, RobotState, Vector2};

fn main() {
    // Create a grass map from an image file
    // - Cell size: 0.01m (1cm)
    // - Simulation ticks: 10 per second
    // - Growth rate: 8mm per day (8.0 / 24.0 / 60.0 / 60.0 mm per second)
    

    // Run simulation for 1 day (in seconds), saving every 600 seconds (10 minutes)
    simulation::start();
}
