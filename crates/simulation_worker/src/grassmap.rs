use image::{ ImageBuffer };
use file_utils::image_loader::load_image_from_path;
// GrassMap.rs - Implements the grass simulation grid for the lawn mower robot
// This file handles the representation, growth, and persistence of the grass environment
use rayon::prelude::*; // For parallel processing of grass cells
// use std::{cmp::min, thread};

use crate::Vector2;

const MAX_GRASS_HEIGHT: f32 = 255.0;
const DEFAULT_GRASS_HEIGHT: f32 = 40.0;

/// Represents the state of a single grass cell in the simulation
#[derive(Clone)]
pub struct GrassInfo {
    pub height: f32,           // Current height of grass in this cell (in millimeters)
    pub last_update_tick: u32, // The last simulation tick when this cell was updated
    pub is_valid: bool, // Whether this cell is valid terrain (false for obstacles/boundaries)
    pub astar: Option<AstarNode>,
}

#[derive(Copy, Clone, PartialEq)]
pub struct AstarNode {
    previous_node: Option<Vector2>,
    cost: f32,
}

/// Represents the entire grass field as a 2D grid
pub struct GrassMap {
    pub size: Vector2,               // Size of the grid in cells (x,y)
    pub data: Vec<Vec<GrassInfo>>,   // 2D grid of grass cells
    pub current_tick: u32,           // Current simulation tick
    pub last_update_tick: u32,       // Last tick when the entire grid was updated
    pub ticks_per_second: u32,       // How many simulation ticks make up one second
    pub growth_rate_per_second: f32, // How fast grass grows in mm/second
}




impl GrassMap {
    /// Creates a new GrassMap with uniform initial grass height
    ///
    /// # Parameters
    /// * `physical_size` - Real-world size of the area in meters
    /// * `physical_cell_size` - Size of each cell in meters
    /// * `ticks_per_second` - Simulation ticks per second
    /// * `growth_rate_per_second` - How fast grass grows in mm/second
    pub fn new(
        physical_size: Vector2,
        physical_cell_size: f32,
        ticks_per_second: u32,
        growth_rate_per_second: f32,
    ) -> GrassMap {
        // Calculate grid size based on physical dimensions and cell size
        let size = Vector2::new(
            (physical_size.x / physical_cell_size).round(),
            (physical_size.y / physical_cell_size).round(),
        );

        // Initialize all grass cells with a default height of 40mm
        let data = vec![
            vec![
                GrassInfo {
                    height: DEFAULT_GRASS_HEIGHT,
                    last_update_tick: 0,
                    is_valid: true,
                    astar: None
                };
                size.x as usize
            ];
            size.y as usize
        ];

        GrassMap {
            size,
            data,
            current_tick: 0,
            last_update_tick: 0,
            ticks_per_second,
            growth_rate_per_second,
        }
    }

    


    /// Creates a new GrassMap from an image file
    ///
    /// The image defines the layout of the grass area where:
    /// - Black pixels (value 0) represent obstacles or invalid areas
    /// - Non-black pixels represent valid grass areas
    ///
    /// # Parameters
    /// * `filepath` - Path to the image file
    /// * `physical_cell_size` - Size of each cell in meters
    /// * `ticks_per_second` - Simulation ticks per second
    /// * `growth_rate_per_second` - How fast grass grows in mm/second
    /// 
    pub fn new_from_image(
        filepath: &str,
        physical_cell_size: f32,
        ticks_per_second: u32,
        growth_rate_per_second: f32,
    ) -> GrassMap {
        // println!("{}", filepath);
        // print!(filepath);
        // Load the image and convert it to a grayscale image
        // let img = image::open(filepath)
        let img: ImageBuffer<image::Luma<u8>, Vec<u8>> = load_image_from_path(filepath)
            .into_luma8();
        let (width, height) = img.dimensions();


        

        // Create a new GrassMap with the size of the image
        let mut grassmap = GrassMap::new(
            Vector2::new(
                width as f32 * physical_cell_size,
                height as f32 * physical_cell_size,
            ),
            physical_cell_size,
            ticks_per_second,
            growth_rate_per_second,
        );

        // Warning if top-left pixel is black (potential issue)
        if img.get_pixel(0, 0)[0] == 0 {
            eprintln!("ERROR: Position (0,0) is not a valid grass cell on the map!");
            eprintln!("The robot cannot start at an invalid position.");
            eprintln!("Please check that the top-left pixel of your map image is not black.");
            std::process::exit(1);
        }

        // Mark black pixels as invalid terrain
        for y in 0..height {
            for x in 0..width {
                let pixel = img.get_pixel(x, y);
                if pixel[0] == 0 {
                    grassmap.data[y as usize][x as usize].is_valid = false;
                    grassmap.data[y as usize][x as usize].height = 0.0;
                }
            }
        }
        grassmap
    }

    /// Updates the height of all grass cells based on elapsed time
    /// Uses parallel processing to accelerate growth calculations
    pub fn grow(&mut self) {
        self.last_update_tick = self.current_tick;

        // If growth rate is 0, just update all cell timestamps without changing heights
        if self.growth_rate_per_second <= 0.0 {
            self.data.par_iter_mut().for_each(|line| {
                line.iter_mut().for_each(|element| {
                    if element.is_valid {
                        element.last_update_tick = self.current_tick;
                    }
                });
            });
            return;
        }

        // Use parallel iterators for better performance
        self.data.par_iter_mut().for_each(|line| {
            line.iter_mut().for_each(|element| {
                if element.is_valid {
                    // Calculate time delta since last update for this cell
                    let delta_time = (self.current_tick - element.last_update_tick) as f32
                        / self.ticks_per_second as f32;
                    element.last_update_tick = self.current_tick;

                    // Apply growth based on time elapsed
                    element.height += self.growth_rate_per_second * delta_time;

                    // Cap grass height at MAX_GRASS_HEIGHT
                    if element.height > MAX_GRASS_HEIGHT {
                        element.height = MAX_GRASS_HEIGHT;
                    }
                }
            });
        });
    }

    /// Checks if a given cell coordinate is within bounds and valid terrain
    ///
    /// # Parameters
    /// * `cell` - The grid coordinates to check
    ///
    /// # Returns
    /// * `true` if the cell is valid and within bounds
    /// * `false` otherwise
    pub fn is_cell_valid(&self, cell: Vector2) -> bool {
        // Check x bounds
        if cell.x < 0.0 || cell.x >= self.size.x {
            return false;
        }
        // Check y bounds
        if cell.y < 0.0 || cell.y >= self.size.y {
            return false;
        }

        // Check if the cell is marked as valid terrain
        if !self.data[cell.y as usize][cell.x as usize].is_valid {
            return false;
        }

        return true;
    }

    /// Saves the current state of the grass map to a compressed file
    ///
    /// # Parameters
    /// * `base_dir` - Directory where the file will be saved
    ///
    /// # Returns
    /// * `Ok(())` on success
    /// * `Err` with IO error on failure
    pub fn save(&mut self, base_dir: &str) -> std::io::Result<()> {
        let start_time = std::time::Instant::now();
        // Create directories if they don't exist
        std::fs::create_dir_all(base_dir)?;

        // Update grass if needed before saving
        if self.current_tick != self.last_update_tick {
            self.grow();
        }
        // Prepare the file path - filename is based on simulation hour
        let hour = self.current_tick / self.ticks_per_second / 3600;
        let filename = format!("{}/{}.zst", base_dir, hour); // Create a zstd compressed file
        let file = std::fs::File::create(&filename)?;
        let mut zstd_encoder = zstd::Encoder::new(file, 3)?; // Compression level 3
        // Include content size in frame header for better compatibility
        zstd_encoder.include_contentsize(true)?; // Prepare data array with dimensions and grass heights
        let x_size = self.size.x as usize;
        let y_size = self.size.y as usize;
        let mut save_data = Vec::with_capacity(y_size * x_size + 2);
        unsafe { save_data.set_len(y_size * x_size + 2) };

        // Store dimensions first (for reading in visualization tools)
        save_data[0] = self.size.y as f32; // Height in cells
        save_data[1] = self.size.x as f32; // Width in cells
        // More elegant solution
        // save_data.extend(self.data.iter()
        //     .flat_map(|line| line.iter())
        //     .map(|element| element.height));
        let mut idx = 2; // Start after dimensions
        for line in &self.data {
            for element in line {
                unsafe {
                    // SAFETY: We ensure idx is within bounds of save_data
                    // by initializing it with the correct capacity
                    *save_data.get_unchecked_mut(idx) = element.height;
                }
                idx += 1; // INCREMENT THE INDEX!
            }
        }

        // Convert float values to bytes for storage
        use std::io::Write;
        // SAFETY: f32 is POD and save_data will not be used after this point.
        let bytes = unsafe {
            std::slice::from_raw_parts(
                save_data.as_ptr() as *const u8,
                save_data.len() * std::mem::size_of::<f32>(),
            )
        };

        // Write and finish the compressed file
        zstd_encoder.write_all(&bytes)?;
        zstd_encoder.finish()?;

        println!(
            "Saved grassmap to {} in {:.2} seconds",
            filename,
            start_time.elapsed().as_secs_f32()
        );
        Ok(())
    }

    /// Saves the current state of the grass map to a compressed file with a custom filename
    ///
    /// # Parameters
    /// * `base_dir` - Directory where the file will be saved
    /// * `filename` - Custom filename for the saved file
    ///
    /// # Returns
    /// * `Ok(())` on success
    /// * `Err` with IO error on failure
    pub fn save_with_filename(&mut self, base_dir: &str, filename: &str) -> std::io::Result<()> {
        let start_time = std::time::Instant::now();
        let mut mowed_cells = 0; // Count of mowed cells for debugging
        // Create directories if they don't exist
        std::fs::create_dir_all(base_dir)?;

        // Update grass if needed before saving
        // BUT ONLY if growth rate > 0 (prevents regrowth when growth is disabled)
        if self.current_tick != self.last_update_tick && self.growth_rate_per_second > 0.0 {
            println!(
                "Growing grass before save (tick {} -> {})",
                self.last_update_tick, self.current_tick
            );
            self.grow();
        } else if self.current_tick != self.last_update_tick {
            // If growth rate is 0, just update the last_update_tick without growing
            println!(
                "Updating tick timestamp without growth (tick {} -> {})",
                self.last_update_tick, self.current_tick
            );
            self.last_update_tick = self.current_tick;

            // Update all cell timestamps to prevent future regrowth issues
            self.data.iter_mut().for_each(|line| {
                line.iter_mut().for_each(|element| {
                    if element.is_valid {
                        element.last_update_tick = self.current_tick;
                    }
                });
            });
        }

        // Prepare the file path with custom filename
        let filepath = format!("{}/{}", base_dir, filename);
        let file = std::fs::File::create(&filepath)?;
        let mut zstd_encoder = zstd::Encoder::new(file, 3)?; // Compression level 3
        // Include content size in frame header for better compatibility
        zstd_encoder.include_contentsize(true)?;

        // Prepare data array with dimensions and grass heights
        let x_size = self.size.x as usize;
        let y_size = self.size.y as usize;
        let mut save_data = Vec::with_capacity(y_size * x_size + 2);
        unsafe { save_data.set_len(y_size * x_size + 2) };

        // Store dimensions first (for reading in visualization tools)
        save_data[0] = self.size.y as f32; // Height in cells
        save_data[1] = self.size.x as f32; // Width in cells

        println!(
            "Saving grassmap dimensions: {}x{} ({} total cells)",
            x_size,
            y_size,
            x_size * y_size
        );

        let mut idx = 2; // Start after dimensions
        for line in &self.data {
            for element in line {
                unsafe {
                    // SAFETY: We ensure idx is within bounds of save_data
                    // by initializing it with the correct capacity
                    *save_data.get_unchecked_mut(idx) = element.height;
                    if element.height < 40.0 {
                        mowed_cells += 1; // Count mowed cells for debugging
                    }
                }
                idx += 1; // INCREMENT THE INDEX!
            }
        }

        // Convert float values to bytes for storage
        use std::io::Write;
        // SAFETY: f32 is POD and save_data will not be used after this point.
        let bytes = unsafe {
            std::slice::from_raw_parts(
                save_data.as_ptr() as *const u8,
                save_data.len() * std::mem::size_of::<f32>(),
            )
        };

        println!("Writing {} bytes to compressed file", bytes.len());

        // Write and finish the compressed file
        zstd_encoder.write_all(&bytes)?;
        zstd_encoder.finish()?;

        println!(
            "Saved grassmap to {} in {:.2} seconds",
            filepath,
            start_time.elapsed().as_secs_f32()
        );
        println!(
            "Total mowed cells: {} ({}% of total)",
            mowed_cells,
            (mowed_cells as f32 / (x_size * y_size) as f32 * 100.0).round()
        );
        Ok(())
    }
}
