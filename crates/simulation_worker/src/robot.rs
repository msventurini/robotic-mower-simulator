// Robot.rs - Implementation of the autonomous lawn mowing robot
// This file defines robot behavior, movement, cutting logic, and battery management
use crate::{AstarNode, GrassMap, Vector2};

/// Defines the movement strategies for the robot
#[derive(Clone, PartialEq)]
pub enum PathMode {
    ZigZagVertical,   // Move in vertical zig-zag pattern (up/down then side)
    ZigZagHorizontal, // Move in horizontal zig-zag pattern (left/right then up/down)
    Random,           // Move in random directions
    Home,             // Move towards home position (for recharging)
    Astar(AstarConfig),
}
#[derive(Clone, PartialEq)]
struct AstarConfig {
    destination: Vector2,                      // Target position to reach
    to_visit_nodes: Vec<(AstarNode, Vector2)>, // Nodes to explore
}

/// Defines the current operational state of the robot
#[derive(PartialEq, Eq, Debug)]
pub enum RobotState {
    Charging,    // Robot is at home base recharging
    Mowing,      // Normal mowing operations
    HomePassive, // Heading home but still cutting grass
    HomeActive,  // Urgently heading home, not cutting grass
}

/// The main robot struct representing the lawn mower
pub struct Robot<'a> {
    pub grassmap: &'a mut GrassMap, // Reference to the grass environment

    pub position: Vector2,      // Current position in grid coordinates
    pub home_position: Vector2, // Home/charging station position in grid coordinates
    pub speed: f32,             // Movement speed in grid cells per second

    pub path_mode: PathMode,            // Current movement strategy
    pub path_mode_aux: PathMode,        // Stored path mode to resume after charging
    pub direction: Vector2,             // Current movement direction
    pub main_zigzag_direction: Vector2, // Main direction for zigzag pattern
    pub aux_zigzag_direction: Vector2,  // Secondary direction for zigzag pattern
    pub aux_direction: Vector2,         // Auxiliary direction for complex movements

    pub cut_radius: f32, // Radius of grass cutting in grid cells
    pub cut_height: f32, // Target height to cut grass to (mm)

    pub battery: f32,                          // Current battery level (0-100%)
    pub battery_walk_consumption: f32,         // Battery drain per second while moving
    pub battery_cut_consumption: f32,          // Battery drain per height difference when cutting
    pub state: RobotState,                     // Current operational state
    pub is_valid_path_zigzag: bool,            // Whether zigzag pattern can be followed
    pub zigzag_walk_aux_distance: f32,         // How far to move in secondary direction
    pub current_zigzag_walk_aux_distance: f32, // Current progress along secondary direction
}

impl<'a> Robot<'a> {
    /// Creates a new robot instance
    ///
    /// # Parameters
    /// * `grassmap` - Reference to the grass environment
    /// * `physical_cell_size_meters` - Size of each grid cell in meters
    /// * `physical_position_meters` - Starting position in meters
    /// * `physical_home_position_meters` - Home base position in meters
    /// * `physical_speed_meters_per_second` - Movement speed in m/s
    /// * `physical_cut_radius_meters` - Cutting radius in meters
    /// * `physical_cut_height_milimeters` - Target grass cutting height in mm
    /// * `battery_walk_consumption` - Battery drain per second while moving (percentage)
    /// * `battery_cut_consumption` - Battery drain per mm cut per second (percentage)
    /// * `path_mode` - Initial movement strategy
    /// * `physical_zigzag_walk_aux_distance` - Zigzag secondary distance in meters
    pub fn new(
        grassmap: &mut GrassMap,
        physical_cell_size_meters: f32,
        physical_position_meters: Vector2,
        physical_home_position_meters: Vector2,
        physical_speed_meters_per_second: f32,
        physical_cut_radius_meters: f32,
        physical_cut_height_milimeters: f32,
        battery_walk_consumption: f32,
        battery_cut_consumption: f32,
        path_mode: PathMode,
        physical_zigzag_walk_aux_distance: f32,
    ) -> Robot {
        // Convert physical measurements to grid coordinates
        let mut temp_rebot = Robot {
            grassmap, // reference to the grassmap
            position: physical_position_meters / physical_cell_size_meters,
            home_position: physical_home_position_meters / physical_cell_size_meters,
            speed: physical_speed_meters_per_second / physical_cell_size_meters,
            cut_radius: physical_cut_radius_meters / physical_cell_size_meters,
            cut_height: physical_cut_height_milimeters,
            battery: 100.0,           // Start with full battery
            battery_walk_consumption, // Battery consumption per second while walking
            battery_cut_consumption,  // Battery consumption per second per cut height diff
            aux_direction: Vector2::ZERO,
            direction: Vector2::ZERO,
            path_mode: path_mode.clone(),
            path_mode_aux: path_mode,  // Store initial path mode
            state: RobotState::Mowing, // Start in mowing state
            is_valid_path_zigzag: true,
            zigzag_walk_aux_distance: physical_zigzag_walk_aux_distance / physical_cell_size_meters,
            main_zigzag_direction: Vector2::ZERO,
            aux_zigzag_direction: Vector2::ZERO,
            current_zigzag_walk_aux_distance: 0.0,
        };

        // Set initial direction based on path mode
        temp_rebot.initialize_direction();

        temp_rebot
    }

    /// Checks if the robot can move one step in the current direction
    pub fn can_walk(&mut self) -> bool {
        // Calculate the next position if we take a step
        let new_position =
            self.position + self.direction * self.speed / self.grassmap.ticks_per_second as f32;

        // Check if the new position is valid (inside bounds and not an obstacle)
        return self.grassmap.is_cell_valid(new_position);
    }

    /// Attempts to move the robot one step in the current direction
    ///
    /// # Returns
    /// * `true` if the robot successfully moved
    /// * `false` if the movement was blocked
    pub fn try_walk(&mut self) -> bool {
        // Calculate distance to move in this tick
        let walk_distance = self.direction * self.speed / self.grassmap.ticks_per_second as f32;
        let new_position = self.position + walk_distance;

        // Check if the new position is valid
        if !self.grassmap.is_cell_valid(new_position) {
            if self.path_mode == PathMode::Home {
                self.position = self.home_position; // If heading home, teleport to home position
                println!(
                    "Stuck heading home at position X:{:?} Y:{:?}, teleporting to home position.",
                    self.position.x, self.position.y
                );
            }
            return false; // Cannot move to invalid position
        }

        // Track distance moved in secondary direction for zigzag patterns
        if (self.path_mode == PathMode::ZigZagHorizontal
            || self.path_mode == PathMode::ZigZagVertical)
            && self.direction == self.aux_zigzag_direction
        {
            self.current_zigzag_walk_aux_distance += walk_distance.length();
        }

        if let PathMode::Astar(x) = &self.path_mode {
            self.change_direction(); // Change direction if using A* pathfinding
        }
        // Update position and consume battery
        self.position = new_position;
        self.battery -= self.battery_walk_consumption / self.grassmap.ticks_per_second as f32;

        return true;
    }

    /// Sets the initial direction based on the current path mode
    pub fn initialize_direction(&mut self) {
        match &self.path_mode {
            PathMode::ZigZagHorizontal => {
                // Start moving left to right
                self.direction = Vector2::new(1.0, 0.0);
                self.main_zigzag_direction = Vector2::new(1.0, 0.0);
                self.aux_zigzag_direction = Vector2::new(0.0, 1.0);
            }
            PathMode::ZigZagVertical => {
                // Start moving top to bottom
                self.direction = Vector2::new(0.0, 1.0);
                self.main_zigzag_direction = Vector2::new(0.0, 1.0);
                self.aux_zigzag_direction = Vector2::new(1.0, 0.0);
            }
            PathMode::Random => {
                // Choose a random direction
                let random_angle = rand::random::<f32>() * std::f32::consts::PI * 2.0;
                if random_angle.cos() != 0.0 && random_angle.sin() != 0.0 {
                    self.direction =
                        Vector2::new(random_angle.cos(), random_angle.sin()).normalize();
                } else {
                    // If we got an invalid direction (exactly 0,0), try again
                    self.initialize_direction();
                }
            }
            PathMode::Home => {
                // Set direction toward home position
                let direction_to_home = self.home_position - self.position;
                self.direction = direction_to_home.normalize();
            }
            PathMode::Astar(vector2) => {}
        }
    }

    /// Changes the robot's direction based on the current path mode
    /// This implements the different movement strategies
    pub fn change_direction(&mut self) {
        match self.path_mode.clone() {
            PathMode::ZigZagHorizontal => {
                if self.direction == self.main_zigzag_direction {
                    // We were moving in main direction, invert it and move in auxiliary
                    self.main_zigzag_direction = self.main_zigzag_direction.inverted();
                    self.direction = self.aux_zigzag_direction;
                } else if self.direction == self.aux_zigzag_direction {
                    // We were moving in auxiliary direction, if blocked invert it
                    if !self.can_walk() {
                        self.current_zigzag_walk_aux_distance = 0.0;
                        self.aux_zigzag_direction = self.aux_zigzag_direction.inverted();
                    }
                    // Switch back to main direction
                    self.direction = self.main_zigzag_direction;
                }
            }
            PathMode::ZigZagVertical => {
                // Same logic as horizontal, but with vertical main direction
                if self.direction == self.main_zigzag_direction {
                    self.main_zigzag_direction = self.main_zigzag_direction.inverted();
                    self.direction = self.aux_zigzag_direction;
                } else if self.direction == self.aux_zigzag_direction {
                    if !self.can_walk() {
                        self.current_zigzag_walk_aux_distance = 0.0;
                        self.aux_zigzag_direction = self.aux_zigzag_direction.inverted();
                    }
                    self.direction = self.main_zigzag_direction;
                }
            }
            PathMode::Random => {
                // Choose a new random direction
                let random_angle = rand::random::<f32>() * std::f32::consts::PI * 2.0;
                if random_angle.cos() != 0.0 && random_angle.sin() != 0.0 {
                    self.direction =
                        Vector2::new(random_angle.cos(), random_angle.sin()).normalize();
                } else {
                    // If we got an invalid direction, try again
                    self.change_direction();
                }
            }
            PathMode::Home => {
                // Update direction to keep pointing toward home
                let direction_to_home = self.home_position - self.position;
                self.direction = direction_to_home.normalize();
            }
            PathMode::Astar(vector2) => {
                let _directions: Vec<Vector2> = vec![
                    Vector2::new(1.0, 0.0),       // Right (0°)
                    Vector2::new(0.707, 0.707),   // Down-Right intermediate (22.5°)
                    Vector2::new(1.0, 1.0),       // Down-Right (45°)
                    Vector2::new(0.383, 0.924),   // Down intermediate (67.5°)
                    Vector2::new(0.0, 1.0),       // Down (90°)
                    Vector2::new(-0.383, 0.924),  // Down-Left intermediate (112.5°)
                    Vector2::new(-1.0, 1.0),      // Down-Left (135°)
                    Vector2::new(-0.707, 0.707),  // Left intermediate (157.5°)
                    Vector2::new(-1.0, 0.0),      // Left (180°)
                    Vector2::new(-0.707, -0.707), // Up-Left intermediate (202.5°)
                    Vector2::new(-1.0, -1.0),     // Up-Left (225°)
                    Vector2::new(-0.383, -0.924), // Up intermediate (247.5°)
                    Vector2::new(0.0, -1.0),      // Up (270°)
                    Vector2::new(0.383, -0.924),  // Up-Right intermediate (292.5°)
                    Vector2::new(1.0, -1.0),      // Up-Right (315°)
                    Vector2::new(0.707, -0.707),  // Right intermediate (337.5°)
                ]
                .iter()
                .map(|v| v.normalize())
                .collect();

                // TODO: Implement A* pathfinding logic using these 16 directions
            }
        }
    }

    /// Cut grass at a specific cell to the target height
    ///
    /// # Parameters
    /// * `cell` - Grid coordinates of the cell to cut
    pub fn cut_cell(&mut self, cell: Vector2) {
        if self.grassmap.is_cell_valid(cell) {
            // Calculate how much grass needs to be cut
            let cut_height_diff =
                self.grassmap.data[cell.y as usize][cell.x as usize].height - self.cut_height;

            // Only cut if grass is taller than target height
            if cut_height_diff > 0.0 {
                // Cut grass to target height
                self.grassmap.data[cell.y as usize][cell.x as usize].height = self.cut_height;

                // Drain battery based on how much grass was cut
                self.battery -= cut_height_diff * self.battery_cut_consumption
                    / self.grassmap.ticks_per_second as f32;
            }
        }
    }

    /// Cut all grass cells within the cutting radius
    /// This simulates the circular cutting blade of the mower
    pub fn cut_circle(&mut self) {
        // Define bounding box around current position (optimization)
        let start_x = (-self.cut_radius + self.position.x).floor() as i32;
        let end_x = (self.cut_radius + self.position.x).ceil() as i32;
        let start_y = (-self.cut_radius + self.position.y).floor() as i32;
        let end_y = (self.cut_radius + self.position.y).ceil() as i32;

        // Check each cell in the bounding box
        for i in start_y..end_y {
            for j in start_x..end_x {
                // Calculate distance from robot center
                let distance = Vector2::new(j as f32, i as f32) - self.position;

                // If within cutting radius, cut the cell
                if distance.length() <= self.cut_radius {
                    self.cut_cell(Vector2::new(j as f32, i as f32));
                }
            }
        }
    }

    pub fn is_home(&self) -> bool {
        // Check if the robot is within 0.5 meters of home position
        let distance_to_home = (self.position - self.home_position).length();
        distance_to_home <= 0.5
    }

    /// Manages the robot's state based on battery level
    ///
    /// # Returns
    /// * Time in seconds to advance the simulation (for charging delay)
    pub fn state_manager(&mut self) -> i32 {
        match self.state {
            RobotState::Mowing => {
                // If battery is between 15-20%, start heading home but keep cutting
                if self.battery <= 20.0 && self.battery >= 15.0 {
                    self.path_mode_aux = self.path_mode.clone(); // Save current mode before switching
                    self.path_mode = PathMode::Home;
                    self.change_direction();
                    self.state = RobotState::HomePassive;
                    println!("Changing to home passive mode.");
                }
                // If battery is below 15%, head home urgently without cutting
                else if self.battery <= 15.0 {
                    self.path_mode_aux = self.path_mode.clone(); // Save current mode before switching
                    self.path_mode = PathMode::Home;
                    self.change_direction();
                    self.state = RobotState::HomeActive;
                    println!("Changing to home active mode.");
                }
                // If battery is depleted, teleport to home (emergency rescue)
                else if self.battery <= 0.0 {
                    self.position = self.home_position;
                    self.state = RobotState::Charging;
                    eprintln!("Battery empty! Teleporting to home position.");
                    // Do NOT return time advance here - let charging happen in next call
                }
            }
            RobotState::HomePassive => {
                // Check if we've reached home
                if self.is_home() {
                    self.state = RobotState::Charging;
                    println!("Reached home. Starting charging.");
                    // Do NOT return time advance here - let charging happen in next call
                }
                // If battery drops further while heading home passively
                else if self.battery <= 15.0 {
                    self.path_mode = PathMode::Home;
                    self.change_direction();
                    self.state = RobotState::HomeActive;
                    println!("Changing to home active mode.");
                }
                // Emergency rescue if battery depleted
                else if self.battery <= 0.0 {
                    println!("Battery empty! Teleporting to home position.");
                    self.position = self.home_position;
                    self.state = RobotState::Charging;
                    // Do NOT return time advance here - let charging happen in next call
                }
            }
            RobotState::HomeActive => {
                // Check if we've reached home
                if self.is_home() {
                    self.state = RobotState::Charging;
                    println!("Reached home. Starting charging.");
                    // Do NOT return time advance here - let charging happen in next call
                }
                // Emergency rescue if battery depleted
                else if self.battery <= 0.0 {
                    println!("Battery empty! Teleporting to home position.");
                    self.position = self.home_position;
                    self.state = RobotState::Charging;
                    // Do NOT return time advance here - let charging happen in next call
                }
            }
            RobotState::Charging => {
                // This is where the actual charging happens
                println!("Charging...");
                self.battery = 100.0; // Fully recharge battery
                self.position = self.home_position; // Ensure position is at home
                self.path_mode = self.path_mode_aux.clone(); // Restore original path mode
                self.initialize_direction(); // Reinitialize direction for the restored path mode
                self.state = RobotState::Mowing; // Resume mowing
                println!(
                    "Charging complete. Returning to {} mode.",
                    match self.path_mode {
                        PathMode::Random => "Random mowing",
                        PathMode::ZigZagHorizontal => "Horizontal zigzag",
                        PathMode::ZigZagVertical => "Vertical zigzag",
                        _ => "mowing",
                    }
                );
                return 3600; // Return time advance ONLY here
            }
        }
        0 // No time advance for normal operations
    }
}
