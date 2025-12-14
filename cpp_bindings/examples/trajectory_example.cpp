/**
 * @file trajectory_example.cpp
 * @brief Example usage of OpenAP TrajectoryPredictor in C++
 *
 * This example demonstrates how to use the TrajectoryPredictor class
 * with your existing AircraftState and route segment data structures.
 *
 * Build:
 *   mkdir build && cd build
 *   cmake .. && make
 *   ./trajectory_example
 */

#include "trajectory_predictor.hpp"

#include <iostream>
#include <iomanip>
#include <vector>

using namespace openap;

// =============================================================================
// Helper: Print prediction results
// =============================================================================

void print_prediction(const TrajectoryPrediction& pred, const std::string& title) {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(70, '=') << "\n";

    std::cout << "Current Phase: " << to_string(pred.current_phase) << "\n";
    std::cout << "Distance Remaining: " << std::fixed << std::setprecision(1)
              << pred.distance_remaining_nm << " NM\n";
    std::cout << "Cruise Altitude: FL" << pred.cruise_altitude_ft / 100 << "\n";

    std::cout << "\nTIMING:\n";
    if (pred.time_to_toc_sec.has_value()) {
        std::cout << "  Time to TOC: "
                  << TrajectoryPrediction::format_time(pred.time_to_toc_sec.value()) << "\n";
    }
    if (pred.time_to_tod_sec.has_value()) {
        std::cout << "  Time to TOD: "
                  << TrajectoryPrediction::format_time(pred.time_to_tod_sec.value()) << "\n";
    }
    std::cout << "  Time to Destination: "
              << TrajectoryPrediction::format_time(pred.time_to_destination_sec) << "\n";

    std::cout << "\nPHASE DURATIONS:\n";
    std::cout << "  Climb:   " << TrajectoryPrediction::format_time(pred.climb_duration_sec) << "\n";
    std::cout << "  Cruise:  " << TrajectoryPrediction::format_time(pred.cruise_duration_sec) << "\n";
    std::cout << "  Descent: " << TrajectoryPrediction::format_time(pred.descent_duration_sec) << "\n";

    if (pred.toc_position.has_value()) {
        std::cout << "\nTOP OF CLIMB:\n";
        std::cout << "  Position: " << std::fixed << std::setprecision(4)
                  << pred.toc_position->latitude << "°N, "
                  << pred.toc_position->longitude << "°E\n";
        if (pred.distance_to_toc_nm.has_value()) {
            std::cout << "  Distance from origin: "
                      << std::setprecision(1) << pred.distance_to_toc_nm.value() << " NM\n";
        }
    }

    if (pred.tod_position.has_value()) {
        std::cout << "\nTOP OF DESCENT:\n";
        std::cout << "  Position: " << std::fixed << std::setprecision(4)
                  << pred.tod_position->latitude << "°N, "
                  << pred.tod_position->longitude << "°E\n";
        if (pred.distance_to_tod_nm.has_value()) {
            std::cout << "  Distance from origin: "
                      << std::setprecision(1) << pred.distance_to_tod_nm.value() << " NM\n";
        }
    }
}

// =============================================================================
// Helper: Create the LTFM -> LFPG route
// =============================================================================

std::vector<RouteSegment> create_ltfm_lfpg_route() {
    // Waypoint data: (identifier, lat, lon, cumulative_distance_nm)
    std::vector<std::tuple<std::string, double, double, double>> waypoint_data = {
        {"LTFM",  41.2753,  28.7519,    0.0},
        {"BARPE", 40.9261,  26.9781,   82.3},
        {"GOLDO", 40.8822,  26.2494,  115.5},
        {"ALX",   40.8550,  25.9572,  128.8},
        {"IDILO", 40.7906,  25.4394,  152.7},
        {"SOSUS", 40.7442,  25.0733,  169.6},
        {"SUTIS", 40.7019,  24.7486,  184.6},
        {"DISOR", 41.2472,  22.7583,  280.5},
        {"ENFAR", 41.7686,  20.5344,  385.3},
        {"RETRA", 42.2283,  19.3350,  445.5},
        {"MADOS", 42.6025,  18.2492,  498.6},
        {"SIPAL", 43.1367,  17.0736,  559.5},
        {"BAXON", 44.4164,  13.4631,  733.8},
        {"SRN",   45.6468,   9.0229,  936.2},
        {"PEPAG", 45.9839,   9.0714,  956.5},
        {"ABESI", 46.1597,   9.0428,  967.1},
        {"UTAVO", 46.4106,   9.0092,  982.3},
        {"ELMUR", 47.1568,   8.9076, 1027.3},
        {"RIPUS", 47.2603,   8.5000, 1045.0},
        {"DITON", 47.3022,   8.3333, 1052.3},
        {"HOC",   47.4666,   7.6654, 1081.1},
        {"MOROK", 47.3966,   6.6555, 1122.4},
        {"PENDU", 47.3489,   6.0326, 1147.9},
        {"JAVVU", 47.3323,   5.8322, 1156.1},
        {"GIVRI", 47.2919,   5.3422, 1176.2},
        {"TINIL", 47.5889,   5.0986, 1196.6},
        {"LFPG",  49.0128,   2.5500, 1329.5},
    };

    // Create waypoints
    std::vector<RouteWaypoint> waypoints;
    waypoints.reserve(waypoint_data.size());

    for (const auto& [id, lat, lon, dist] : waypoint_data) {
        RouteWaypoint wpt(id, lat, lon, dist);
        waypoints.push_back(wpt);
    }

    // Create segments from waypoints
    return create_route_from_waypoints(waypoints);
}

// =============================================================================
// Main
// =============================================================================

int main() {
    std::cout << std::string(70, '=') << "\n";
    std::cout << "OPENAP C++ TRAJECTORY PREDICTION EXAMPLE\n";
    std::cout << "Flight: LTFM (Istanbul) -> LFPG (Paris CDG)\n";
    std::cout << "Aircraft: B738 (Boeing 737-800)\n";
    std::cout << std::string(70, '=') << "\n";

    try {
        // Initialize the predictor
        std::cout << "\nInitializing TrajectoryPredictor for B738...\n";
        TrajectoryPredictor predictor("B738");

        if (!predictor.is_ready()) {
            std::cerr << "ERROR: Predictor failed to initialize\n";
            return 1;
        }

        // Get and display aircraft performance
        AircraftPerformance perf = predictor.get_performance();
        std::cout << "\nAircraft Performance:\n";
        std::cout << "  Climb CAS: " << perf.climb_cas_kts << " kt\n";
        std::cout << "  Climb Mach: " << perf.climb_mach << "\n";
        std::cout << "  Cruise Mach: " << perf.cruise_mach << "\n";
        std::cout << "  Descent V/S: " << perf.descent_vs_fpm << " fpm\n";

        // Create the route
        std::vector<RouteSegment> route = create_ltfm_lfpg_route();
        std::cout << "\nRoute created with " << route.size() << " segments\n";

        const int cruise_altitude_ft = 37000;  // FL370

        // =====================================================================
        // SCENARIO 1: Aircraft on ground at LTFM
        // =====================================================================
        {
            AircraftState state;
            state.position = Position(41.2753, 28.7519);  // LTFM
            state.pressure_altitude_ft = 0;
            state.ground_speed_kts = 0;
            state.track_heading = 0;
            state.vertical_speed_fpm = 0;
            state.is_on_ground = true;

            TrajectoryPrediction pred = predictor.predict(state, route, cruise_altitude_ft);
            print_prediction(pred, "SCENARIO 1: Aircraft on ground at LTFM");
        }

        // =====================================================================
        // SCENARIO 2: Aircraft climbing through FL200
        // =====================================================================
        {
            AircraftState state;
            state.position = Position(40.95, 27.2);  // Between LTFM and BARPE
            state.pressure_altitude_ft = 20000;
            state.ground_speed_kts = 420;
            state.track_heading = 280;
            state.vertical_speed_fpm = 1800;  // Climbing
            state.is_on_ground = false;

            TrajectoryPrediction pred = predictor.predict(state, route, cruise_altitude_ft);
            print_prediction(pred, "SCENARIO 2: Aircraft climbing through FL200");
        }

        // =====================================================================
        // SCENARIO 3: Aircraft in cruise at FL370 near BAXON
        // =====================================================================
        {
            AircraftState state;
            state.position = Position(44.4164, 13.4631);  // BAXON
            state.pressure_altitude_ft = 37000;
            state.ground_speed_kts = 453;
            state.track_heading = 290;
            state.vertical_speed_fpm = 0;  // Level
            state.is_on_ground = false;

            TrajectoryPrediction pred = predictor.predict(state, route, cruise_altitude_ft);
            print_prediction(pred, "SCENARIO 3: Aircraft in cruise at FL370");
        }

        // =====================================================================
        // SCENARIO 4: Aircraft descending through FL250
        // =====================================================================
        {
            AircraftState state;
            state.position = Position(48.0, 4.5);  // Past TINIL
            state.pressure_altitude_ft = 25000;
            state.ground_speed_kts = 380;
            state.track_heading = 320;
            state.vertical_speed_fpm = -1500;  // Descending
            state.is_on_ground = false;

            TrajectoryPrediction pred = predictor.predict(state, route, cruise_altitude_ft);
            print_prediction(pred, "SCENARIO 4: Aircraft descending through FL250");
        }

        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "All scenarios completed successfully!\n";
        std::cout << std::string(70, '=') << "\n";

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
