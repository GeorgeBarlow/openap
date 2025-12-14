# OpenAP C++ Bindings

C++ bindings for OpenAP trajectory prediction using pybind11.

## Overview

This module provides a C++ interface to OpenAP's trajectory prediction capabilities. It allows you to:

- Predict time to Top of Climb (TOC) and Top of Descent (TOD)
- Calculate estimated time to destination
- Get geographic positions for TOC/TOD points
- Handle aircraft in any flight phase (ground, climb, cruise, descent)

## Requirements

- **C++17** compiler (GCC 8+, Clang 7+, MSVC 2019+)
- **CMake** 3.14+
- **Python** 3.8+ with development headers
- **pybind11** (automatically fetched if not found)
- **OpenAP** Python package (installed in your Python environment)

## Quick Start

### 1. Install OpenAP

```bash
cd /path/to/openap
pip install -e .
```

### 2. Build the C++ Bindings

```bash
cd cpp_bindings
mkdir build && cd build
cmake ..
make
```

### 3. Run the Example

```bash
./trajectory_example
```

## Usage in Your Project

### CMake Integration

```cmake
# In your CMakeLists.txt
find_package(openap_trajectory REQUIRED)

add_executable(your_app main.cpp)
target_link_libraries(your_app PRIVATE openap::openap_trajectory)
```

Or add as a subdirectory:

```cmake
add_subdirectory(path/to/openap/cpp_bindings)
target_link_libraries(your_app PRIVATE openap_trajectory)
```

### Code Example

```cpp
#include "trajectory_predictor.hpp"
#include <iostream>

int main() {
    // Initialize predictor for Boeing 737-800
    openap::TrajectoryPredictor predictor("B738");

    // Define aircraft state
    openap::AircraftState state;
    state.position = openap::Position(41.2753, 28.7519);  // LTFM
    state.pressure_altitude_ft = 0;
    state.ground_speed_kts = 0;
    state.vertical_speed_fpm = 0;
    state.is_on_ground = true;

    // Define route (abbreviated)
    std::vector<openap::RouteWaypoint> waypoints = {
        {"LTFM",  41.2753, 28.7519,    0.0},
        {"BARPE", 40.9261, 26.9781,   82.3},
        {"GOLDO", 40.8822, 26.2494,  115.5},
        // ... more waypoints ...
        {"LFPG",  49.0128,  2.5500, 1329.5},
    };

    auto route = openap::create_route_from_waypoints(waypoints);

    // Predict trajectory
    auto prediction = predictor.predict(state, route, 37000);  // FL370

    // Access results
    std::cout << "Time to TOC: "
              << openap::TrajectoryPrediction::format_time(
                     prediction.time_to_toc_sec.value())
              << std::endl;

    std::cout << "Time to destination: "
              << openap::TrajectoryPrediction::format_time(
                     prediction.time_to_destination_sec)
              << std::endl;

    if (prediction.toc_position) {
        std::cout << "TOC Position: "
                  << prediction.toc_position->latitude << "°N, "
                  << prediction.toc_position->longitude << "°E"
                  << std::endl;
    }

    return 0;
}
```

## Data Structures

### AircraftState

```cpp
struct AircraftState {
    Position position;              // lat/lon
    int pressure_altitude_ft;       // feet
    double ground_speed_kts;        // knots
    int track_heading;              // degrees (0-360)
    int reported_heading;           // degrees
    int vertical_speed_fpm;         // feet per minute (+up/-down)
    bool is_on_ground;

    FlightPhase get_phase(int threshold_fpm = 300) const;
};
```

### RouteWaypoint

```cpp
struct RouteWaypoint {
    std::string identifier;                   // e.g., "BARPE"
    Position position;                        // lat/lon
    double distance_from_origin_nm;           // cumulative distance
    std::optional<int> planned_altitude_ft;   // optional
    std::optional<int> planned_speed_kts;     // optional
};
```

### TrajectoryPrediction

```cpp
struct TrajectoryPrediction {
    // Times (seconds from now)
    std::optional<double> time_to_toc_sec;
    std::optional<double> time_to_tod_sec;
    double time_to_destination_sec;

    // Distances (nautical miles)
    std::optional<double> distance_to_toc_nm;
    std::optional<double> distance_to_tod_nm;
    double distance_remaining_nm;

    // Positions
    std::optional<Position> toc_position;
    std::optional<Position> tod_position;

    // Phase durations
    double climb_duration_sec;
    double cruise_duration_sec;
    double descent_duration_sec;

    // Current status
    FlightPhase current_phase;
    int cruise_altitude_ft;
};
```

## Supported Aircraft Types

OpenAP includes kinematic models for:

| Code | Aircraft |
|------|----------|
| A319 | Airbus A319 |
| A320 | Airbus A320 |
| A321 | Airbus A321 |
| A332 | Airbus A330-200 |
| A333 | Airbus A330-300 |
| A343 | Airbus A340-300 |
| A388 | Airbus A380-800 |
| B737 | Boeing 737-700 |
| B738 | Boeing 737-800 |
| B739 | Boeing 737-900 |
| B744 | Boeing 747-400 |
| B752 | Boeing 757-200 |
| B763 | Boeing 767-300 |
| B77W | Boeing 777-300ER |
| B788 | Boeing 787-8 |
| B789 | Boeing 787-9 |
| E190 | Embraer E190 |

Other aircraft types may use synonym mapping to the closest model.

## Thread Safety

The `TrajectoryPredictor` class uses Python's GIL (Global Interpreter Lock) for thread safety. Multiple instances can exist simultaneously, but predictions are serialized through the GIL.

For high-performance multi-threaded applications, consider:
- Using a thread pool with one predictor per thread
- Caching common predictions
- Running predictions asynchronously

## Integration with Your Existing Code

If you have existing code using structures like `erkir::spherical::Point`, you can easily adapt:

```cpp
// Your existing struct
struct YourAircraftState {
    erkir::spherical::Point position;
    int pressureAltitude;
    double groundSpeedKnots;
    int trackHeading;
    int reportedHeading;
    int verticalSpeed;
    bool isOnGround;
};

// Conversion function
openap::AircraftState convert(const YourAircraftState& your_state) {
    openap::AircraftState state;
    state.position = openap::Position(
        your_state.position.latitude(),
        your_state.position.longitude()
    );
    state.pressure_altitude_ft = your_state.pressureAltitude;
    state.ground_speed_kts = your_state.groundSpeedKnots;
    state.track_heading = your_state.trackHeading;
    state.reported_heading = your_state.reportedHeading;
    state.vertical_speed_fpm = your_state.verticalSpeed;
    state.is_on_ground = your_state.isOnGround;
    return state;
}
```

## Troubleshooting

### Python not found

Ensure Python development headers are installed:

```bash
# Ubuntu/Debian
sudo apt install python3-dev

# Fedora
sudo dnf install python3-devel

# macOS
brew install python3
```

### OpenAP module not found

Make sure OpenAP is installed in the Python environment being used:

```bash
pip install openap
# or
pip install -e /path/to/openap
```

### pybind11 not found

pybind11 will be automatically downloaded if not found. If you prefer to install it:

```bash
pip install pybind11
# or
sudo apt install pybind11-dev
```

## License

This code follows the same license as OpenAP (GNU Lesser General Public License v3.0).
