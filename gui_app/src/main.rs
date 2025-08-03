// Main.rs - Entry point for the lawn mowing robot simulation
// This file sets up the simulation parameters and controls the main simulation loop
use std::time::Instant;

use gui_app::{GrassMap, PathMode, Robot, RobotState, Vector2};

/// Main simulation function that runs the robot for a specified duration
///
/// # Parameters
/// * `robot` - The robot instance to simulate
/// * `runtime` - Total simulation time in seconds
/// * `save_interval` - How often to save the grassmap (in seconds)
fn run(mut robot: Robot, runtime: i32, save_interval: i32) {
    let start_time = Instant::now(); // Start the timer for whole simulation
    let mut iter_time = Instant::now(); // Start the timer for each iteration
    let mut last_save_seconds = 0; // Last time we saved the grassmap

    // Main simulation loop - each iteration is one second of simulated time
    for mut seconds in 0..=runtime {
        // Handle state changes that might advance time (like charging)
        let time_advance = robot.state_manager();
        seconds += time_advance;

        // DEBUG: Print state information during charging
        if time_advance > 0 {
            println!(
                "DEBUG: Time advanced by {} seconds, robot state: {:?}, position: ({:.2}, {:.2})",
                time_advance, robot.state, robot.position.x, robot.position.y
            );
        }

        // Update grassmap current_tick to match the advanced time
        if time_advance > 0 {
            robot.grassmap.current_tick += (time_advance as u32) * robot.grassmap.ticks_per_second;
        }

        // Inner loop - process each tick within the current second
        // CRITICAL: Skip ALL tick processing if robot charged (time was advanced)
        if time_advance == 0 {
            for _ in 0..robot.grassmap.ticks_per_second {
                // Try to move forward, change direction if blocked
                if !robot.try_walk() {
                    robot.change_direction();
                };

                // Check if we've moved far enough in the zigzag auxiliary direction
                if robot.current_zigzag_walk_aux_distance > robot.zigzag_walk_aux_distance {
                    robot.current_zigzag_walk_aux_distance = 0.0;
                    robot.change_direction();
                }

                // Cut grass ONLY if robot is in normal mowing state
                // This prevents cutting during ANY state transitions
                if robot.state == RobotState::Mowing {
                    robot.cut_circle();
                }

                // Advance simulation time
                robot.grassmap.current_tick += 1;
            }
        } else {
            // DEBUG: Confirm we're skipping tick processing
            println!("DEBUG: Skipping tick processing during charging period");
        }

        // Save the grassmap state every save_interval seconds
        if (seconds - last_save_seconds) >= save_interval && seconds <= runtime {
            // If there was a time advance (e.g., charging), generate intermediate saves
            if time_advance > 0 {
                // Generate saves for each missed interval during the time jump
                let mut save_time = last_save_seconds + save_interval;
                while save_time <= seconds - save_interval {
                    println!(
                        "Saving grassmap at {} seconds (during charging) - Robot at ({:.2}, {:.2}), Battery: {:.2}%",
                        save_time, robot.position.x, robot.position.y, robot.battery
                    );

                    let filename = format!("{}.zst", save_time);
                    match robot.grassmap.save_with_filename("./results/", &filename) {
                        Ok(()) => println!("Successfully saved {}", filename),
                        Err(e) => eprintln!("Failed to save {}: {}", filename, e),
                    }
                    last_save_seconds = save_time;
                    save_time += save_interval;
                }
            }

            // Save the current state
            println!(
                "Saving grassmap at {} seconds - Robot at ({:.2}, {:.2}), Battery: {:.2}%",
                seconds, robot.position.x, robot.position.y, robot.battery
            );
            println!(
                "Grassmap tick: {}, Last update tick: {}",
                robot.grassmap.current_tick, robot.grassmap.last_update_tick
            );
            println!(
                "Last simulation cycle: {} seconds",
                (Instant::now() - iter_time).as_secs_f32()
            );
            iter_time = Instant::now();

            let filename = format!("{}.zst", seconds);
            match robot.grassmap.save_with_filename("./results/", &filename) {
                Ok(()) => println!("Successfully saved {}", filename),
                Err(e) => eprintln!("Failed to save {}: {}", filename, e),
            }
            last_save_seconds = seconds;
        }
    }

    // Report total simulation time
    println!(
        "Total time elapsed: {} seconds",
        (Instant::now() - start_time).as_secs_f32()
    );
}

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
    run(robot, 1 * 24 * 3600, 600);
}
