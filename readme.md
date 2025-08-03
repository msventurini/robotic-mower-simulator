# Robotic Lawn Mower Simulation

A comprehensive simulation framework for analyzing the performance of autonomous robotic lawn mowers with different path
planning strategies and grass growth scenarios.

## 🎯 Project Overview

This project simulates robotic lawn mowers to study how different factors affect their performance:
- **Grass growth rates** and their impact on mowing efficiency
- **Path planning strategies** (zigzag vs. random movement)
- **Battery consumption** patterns during operation
- **Coverage optimization** for maintaining lawn quality

The simulation generates data that helps determine the optimal mowing frequency and strategy for maintaining lawns
within specific height and quality parameters.

## 🔬 Research Context

This work is part of a Master's thesis research project that:
- Validates simulation results against real-world data collected from actual robotic mowers
- Analyzes the relationship between grass height, battery consumption, and mowing performance
- Proposes new algorithms for minimizing grass trampling while maximizing coverage
- Aims to optimize weekly mowing schedules for different lawn conditions

## 🚀 Features

### Simulation Engine
- **Realistic grass growth modeling** with configurable growth rates
- **Multi-strategy robot behavior**: zigzag patterns, random movement, and intelligent home-seeking
- **Battery management system** with realistic consumption patterns
- **Parallel processing** for efficient large-area simulations
- **Image-based terrain loading** for custom lawn shapes and obstacles

### Data Collection & Analysis
- **Compressed data output** using zstandard for efficient storage
- **Comprehensive metrics tracking**: coverage, battery usage, time efficiency
- **Visualization tools** for analyzing simulation results
- **Export capabilities** for further statistical analysis

### Path Planning Algorithms
- **Zigzag pattern**: Systematic coverage with predictable paths
- **Random movement**: Probabilistic coverage with natural behavior
- **Trampling minimizing**: Path planning minimizing grass trampling and optimizing coverage
- **Obstacle avoidance**: Smart navigation around lawn features

## 🛠️ Technical Architecture

### Core Components
- **`vector2.rs`**: 2D vector mathematics for position and movement calculations
- **`grassmap.rs`**: Grid-based grass simulation with growth and cutting mechanics
- **`robot.rs`**: Robot behavior, state management, and decision-making logic
- **`main.rs`**: Simulation orchestration and parameter management

### Key Technologies
- **Rust**: High-performance simulation engine with memory safety
- **Parallel processing**: Multi-threaded grass updates for large areas
- **Zstandard compression**: Efficient data storage and transmission
- **Python visualization**: Jupyter notebooks for data analysis and plotting

## 📊 Data & Validation

The simulation is validated against real-world data including:
- **Actual robot drive paths** recorded from garden deployments
- **Coverage measurements** over extended time periods
- **Battery consumption data** for different grass heights and conditions

## 🎯 Research Goals

### Primary Objectives
1. **Optimize mowing frequency**: Determine ideal weekly schedules for different grass types
2. **Compare path strategies**: Evaluate efficiency of different movement patterns
3. **Minimize grass damage**: Develop algorithms that reduce trampling

### Major milestones/To Do
- Analysis from real-world data to validate simulation results
- Run simulations ignoring grass growth and compare with real-world data
  - 1st task: Make simulation match real data for one map
  - 2nd task: Run simulation for all real data maps
- Analyse real world battery consumption data per grass height
- Implement battery consumption models based on grass height and collected data
- Determine a lawn quality indicator based on grass height and its gradients
- Use the simulation to determine optimal mowing frequency for a professional FIFA soccer field
- Compare zigzag and random movement strategies in terms of coverage and efficiency, estimating the maximum area that
can be kept within a specific grass quality range
- Write and publish a scientific paper based on the findings
- Development of path planning algorithms that minimize grass trampling while maximizing coverage
- Extend paper with the planning algorithms and publish the thesis.


## 🚀 Getting Started

### Prerequisites
- Rust (latest stable version)
- Python 3.7+ with Jupyter notebook support
- Required Python packages: `matplotlib`, `numpy`, `zstandard`

### Running the Simulation
```bash
# Clone the repository
git clone [repository-url]
cd mestrado-rust

# Run the simulation
cargo run --release

# Analyze results
jupyter notebook visualize_grass.ipynb
```

### Configuration
Modify simulation parameters in `main.rs`:
- Grid size and resolution
- Grass growth rates
- Robot behavior settings
- Simulation duration and output frequency

## 📈 Output & Analysis

The simulation generates:
- **Time-series data** of grass heights across the lawn
- **Robot movement traces** showing coverage patterns
- **Battery consumption logs** for performance analysis
- **Statistical summaries** of efficiency metrics

Results are saved in compressed format and can be visualized using the provided Jupyter notebook.

## 🔬 Academic Impact

This research contributes to:
- **Autonomous robotics**: Improved path planning for service robots
- **Agricultural technology**: Data-driven lawn maintenance strategies
- **Sustainability**: Optimized energy usage in robotic systems
- **Computer simulation**: Validated models for real-world robot behavior

## 📝 Publications

This work supports ongoing research and thesis development in robotic lawn maintenance optimization and autonomous
vehicle path planning.

---

*This project is part of ongoing Master's thesis research in autonomous robotic systems and agricultural technology
optimization.*
