// Vector2.rs - A 2D vector implementation for the grass cutting robot simulation
// This file provides a complete 2D vector struct with common vector operations
// needed for robot movement and spatial calculations.
use std::ops::{Add, Div, Mul, Sub};

/// A 2D vector with x and y components, used for positions, directions, and velocities
/// throughout the simulation.
#[derive(Clone, Copy, PartialEq)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

impl Vector2 {
    /// Constant representing the zero vector (0,0)
    pub const ZERO: Vector2 = Vector2 { x: 0.0, y: 0.0 };

    /// Creates a new Vector2 with the given x and y components
    pub fn new(x: f32, y: f32) -> Vector2 {
        Vector2 { x, y }
    }

    /// Calculates the length (magnitude) of the vector using the Pythagorean theorem
    pub fn length(&self) -> f32 {
        f32::sqrt(f32::powi(self.x, 2) + f32::powi(self.y, 2))
    }

    /// Returns a normalized version of the vector (same direction, but length of 1)
    /// Returns the zero vector if the original vector has zero length.
    pub fn normalize(&self) -> Vector2 {
        let length = self.length();

        match length {
            0.0 => Vector2::ZERO, // Prevent division by zero
            _ => Vector2::new(self.x / length, self.y / length),
        }
    }

    /// Returns the angle of the vector in radians (from the positive x-axis)
    pub fn angle(&self) -> f32 {
        f32::atan2(self.y, self.x)
    }

    /// Rotates this vector by the angle represented by another vector
    /// Uses standard 2D rotation matrix transformation
    pub fn rotate_by(&self, other_vector: Vector2) -> Vector2 {
        let angle = other_vector.angle();

        Vector2::new(
            f32::cos(angle) * self.x - f32::sin(angle) * self.y,
            f32::sin(angle) * self.x + f32::cos(angle) * self.y,
        )
    }

    /// Returns a vector perpendicular (90° counterclockwise rotation) to this one
    pub fn perpendicular(&self) -> Vector2 {
        Vector2::new(-self.y, self.x)
    }

    /// Calculates a normalized direction vector pointing from this vector to another
    pub fn direction_to(&self, other_vector: Vector2) -> Vector2 {
        Vector2::new(other_vector.x - self.x, other_vector.y - self.y).normalize()
    }

    /// Returns a new vector with components rounded to the nearest integer
    pub fn rounded(&self) -> Vector2 {
        Vector2::new(self.x.round(), self.y.round())
    }

    /// Returns a vector with the same magnitude but opposite direction
    pub fn inverted(&self) -> Vector2 {
        Vector2::new(-self.x, -self.y)
    }
}

// Operator overloading implementations to allow natural vector arithmetic

/// Implements vector addition (v1 + v2)
impl Add<Vector2> for Vector2 {
    type Output = Vector2;

    fn add(self, other_vector: Vector2) -> Self::Output {
        Vector2::new(self.x + other_vector.x, self.y + other_vector.y)
    }
}

/// Implements vector subtraction (v1 - v2)
impl Sub<Vector2> for Vector2 {
    type Output = Vector2;

    fn sub(self, other_vector: Vector2) -> Self::Output {
        Vector2::new(self.x - other_vector.x, self.y - other_vector.y)
    }
}

/// Implements scalar multiplication (vector * scalar)
impl Mul<f32> for Vector2 {
    type Output = Vector2;

    fn mul(self, multiplier: f32) -> Self::Output {
        Vector2::new(self.x * multiplier, self.y * multiplier)
    }
}

/// Implements scalar division (vector / scalar)
impl Div<f32> for Vector2 {
    type Output = Vector2;

    fn div(self, divisor: f32) -> Self::Output {
        Vector2::new(self.x / divisor, self.y / divisor)
    }
}

/// Implements conversion from a tuple to Vector2
impl From<(f32, f32)> for Vector2 {
    fn from(value: (f32, f32)) -> Vector2 {
        Vector2::new(value.0, value.1)
    }
}
