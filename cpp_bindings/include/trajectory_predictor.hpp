/**
 * @file trajectory_predictor.hpp
 * @brief C++ interface for OpenAP trajectory prediction
 *
 * This header provides C++ data structures and the TrajectoryPredictor class
 * that wraps the Python OpenAP library via pybind11.
 *
 * Usage:
 *   #include "trajectory_predictor.hpp"
 *
 *   openap::TrajectoryPredictor predictor("B738");
 *   auto prediction = predictor.predict(state, route, 37000);
 */

#pragma once

#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <stdexcept>

namespace openap {

// =============================================================================
// Data Structures
// =============================================================================

/**
 * @brief Geographic position in WGS84 coordinates
 */
struct Position {
    double latitude;   ///< Latitude in degrees (-90 to +90)
    double longitude;  ///< Longitude in degrees (-180 to +180)

    Position() : latitude(0.0), longitude(0.0) {}
    Position(double lat, double lon) : latitude(lat), longitude(lon) {}
};

/**
 * @brief Flight phase enumeration
 */
enum class FlightPhase {
    GROUND,
    CLIMB,
    CRUISE,
    DESCENT
};

/**
 * @brief Convert FlightPhase to string
 */
inline std::string to_string(FlightPhase phase) {
    switch (phase) {
        case FlightPhase::GROUND:  return "ground";
        case FlightPhase::CLIMB:   return "climb";
        case FlightPhase::CRUISE:  return "cruise";
        case FlightPhase::DESCENT: return "descent";
        default: return "unknown";
    }
}

/**
 * @brief Parse FlightPhase from string
 */
inline FlightPhase phase_from_string(const std::string& str) {
    if (str == "ground")  return FlightPhase::GROUND;
    if (str == "climb")   return FlightPhase::CLIMB;
    if (str == "cruise")  return FlightPhase::CRUISE;
    if (str == "descent") return FlightPhase::DESCENT;
    return FlightPhase::GROUND;
}

/**
 * @brief Current aircraft state
 *
 * This structure mirrors your existing AircraftState struct.
 * All values use aviation-standard units.
 */
struct AircraftState {
    Position position;              ///< Current lat/lon position
    int pressure_altitude_ft;       ///< Pressure altitude in feet
    double ground_speed_kts;        ///< Ground speed in knots
    int track_heading;              ///< Track (direction of travel) in degrees
    int reported_heading;           ///< Nose heading in degrees
    int vertical_speed_fpm;         ///< Vertical speed in feet per minute (+up/-down)
    bool is_on_ground;              ///< True if aircraft is on the ground

    AircraftState()
        : position()
        , pressure_altitude_ft(0)
        , ground_speed_kts(0.0)
        , track_heading(0)
        , reported_heading(0)
        , vertical_speed_fpm(0)
        , is_on_ground(true) {}

    /**
     * @brief Determine current flight phase from vertical speed
     * @param threshold_fpm Vertical speed threshold (default 300 fpm)
     */
    FlightPhase get_phase(int threshold_fpm = 300) const {
        if (is_on_ground) return FlightPhase::GROUND;
        if (vertical_speed_fpm > threshold_fpm) return FlightPhase::CLIMB;
        if (vertical_speed_fpm < -threshold_fpm) return FlightPhase::DESCENT;
        return FlightPhase::CRUISE;
    }
};

/**
 * @brief Route waypoint with optional planned altitude/speed
 */
struct RouteWaypoint {
    std::string identifier;         ///< Waypoint identifier (e.g., "BARPE")
    Position position;              ///< Waypoint position
    double distance_from_origin_nm; ///< Cumulative distance from departure
    std::optional<int> planned_altitude_ft;  ///< Planned altitude at waypoint
    std::optional<int> planned_speed_kts;    ///< Planned speed at waypoint

    RouteWaypoint()
        : identifier("")
        , position()
        , distance_from_origin_nm(0.0)
        , planned_altitude_ft(std::nullopt)
        , planned_speed_kts(std::nullopt) {}

    RouteWaypoint(const std::string& id, double lat, double lon, double dist = 0.0)
        : identifier(id)
        , position(lat, lon)
        , distance_from_origin_nm(dist)
        , planned_altitude_ft(std::nullopt)
        , planned_speed_kts(std::nullopt) {}
};

/**
 * @brief Route segment connecting two waypoints
 */
struct RouteSegment {
    RouteWaypoint from;
    RouteWaypoint to;
    std::string airway;     ///< Airway name or "DCT" for direct
    double distance_nm;     ///< Segment distance in nautical miles

    RouteSegment()
        : from()
        , to()
        , airway("DCT")
        , distance_nm(0.0) {}

    RouteSegment(const RouteWaypoint& f, const RouteWaypoint& t, double dist)
        : from(f)
        , to(t)
        , airway("DCT")
        , distance_nm(dist) {}
};

/**
 * @brief Complete trajectory prediction result
 */
struct TrajectoryPrediction {
    // Time predictions (seconds from now)
    std::optional<double> time_to_toc_sec;      ///< Time to top of climb
    std::optional<double> time_to_tod_sec;      ///< Time to top of descent
    double time_to_destination_sec;             ///< Time to destination

    // Distance predictions (nautical miles)
    std::optional<double> distance_to_toc_nm;   ///< Distance to TOC from current position
    std::optional<double> distance_to_tod_nm;   ///< Distance to TOD from origin
    double distance_remaining_nm;               ///< Total remaining distance

    // Position predictions
    std::optional<Position> toc_position;       ///< Top of climb lat/lon
    std::optional<Position> tod_position;       ///< Top of descent lat/lon

    // Phase durations (seconds)
    double climb_duration_sec;
    double cruise_duration_sec;
    double descent_duration_sec;

    // Altitudes
    int cruise_altitude_ft;

    // Current status
    FlightPhase current_phase;

    TrajectoryPrediction()
        : time_to_toc_sec(std::nullopt)
        , time_to_tod_sec(std::nullopt)
        , time_to_destination_sec(0.0)
        , distance_to_toc_nm(std::nullopt)
        , distance_to_tod_nm(std::nullopt)
        , distance_remaining_nm(0.0)
        , toc_position(std::nullopt)
        , tod_position(std::nullopt)
        , climb_duration_sec(0.0)
        , cruise_duration_sec(0.0)
        , descent_duration_sec(0.0)
        , cruise_altitude_ft(0)
        , current_phase(FlightPhase::GROUND) {}

    /**
     * @brief Format time as HH:MM:SS string
     */
    static std::string format_time(double seconds) {
        int hrs = static_cast<int>(seconds) / 3600;
        int mins = (static_cast<int>(seconds) % 3600) / 60;
        int secs = static_cast<int>(seconds) % 60;

        char buf[32];
        if (hrs > 0) {
            snprintf(buf, sizeof(buf), "%dh %02dm %02ds", hrs, mins, secs);
        } else {
            snprintf(buf, sizeof(buf), "%dm %02ds", mins, secs);
        }
        return std::string(buf);
    }
};

/**
 * @brief Aircraft performance characteristics from OpenAP
 */
struct AircraftPerformance {
    std::string aircraft_type;

    // Climb
    double climb_cas_kts;
    double climb_mach;
    double climb_vs_fpm;

    // Cruise
    double cruise_mach;
    double cruise_alt_ft;

    // Descent
    double descent_cas_kts;
    double descent_mach;
    double descent_vs_fpm;
};

// =============================================================================
// TrajectoryPredictor Class (Forward Declaration)
// =============================================================================

// Implementation details hidden in cpp file
class TrajectoryPredictorImpl;

/**
 * @brief Trajectory predictor using OpenAP kinematic models
 *
 * This class provides trajectory prediction capabilities using the OpenAP
 * Python library via pybind11 embedded Python.
 *
 * Example usage:
 * @code
 *   openap::TrajectoryPredictor predictor("B738");
 *
 *   openap::AircraftState state;
 *   state.position = {41.2753, 28.7519};
 *   state.pressure_altitude_ft = 0;
 *   state.is_on_ground = true;
 *
 *   std::vector<openap::RouteSegment> route = create_route();
 *
 *   auto prediction = predictor.predict(state, route, 37000);
 *   std::cout << "Time to destination: "
 *             << prediction.format_time(prediction.time_to_destination_sec)
 *             << std::endl;
 * @endcode
 */
class TrajectoryPredictor {
public:
    /**
     * @brief Construct predictor for specific aircraft type
     * @param aircraft_type ICAO aircraft type code (e.g., "B738", "A320", "B77W")
     * @throws std::runtime_error if aircraft type not supported or Python init fails
     */
    explicit TrajectoryPredictor(const std::string& aircraft_type);

    /**
     * @brief Destructor
     */
    ~TrajectoryPredictor();

    // Non-copyable
    TrajectoryPredictor(const TrajectoryPredictor&) = delete;
    TrajectoryPredictor& operator=(const TrajectoryPredictor&) = delete;

    // Movable
    TrajectoryPredictor(TrajectoryPredictor&&) noexcept;
    TrajectoryPredictor& operator=(TrajectoryPredictor&&) noexcept;

    /**
     * @brief Generate trajectory prediction
     * @param state Current aircraft state
     * @param route Route segments from departure to destination
     * @param cruise_altitude_ft Target cruise altitude in feet
     * @return TrajectoryPrediction with timing and position estimates
     */
    TrajectoryPrediction predict(
        const AircraftState& state,
        const std::vector<RouteSegment>& route,
        int cruise_altitude_ft
    );

    /**
     * @brief Get aircraft performance characteristics
     * @return AircraftPerformance with climb/cruise/descent parameters
     */
    AircraftPerformance get_performance() const;

    /**
     * @brief Get the aircraft type code
     */
    const std::string& aircraft_type() const;

    /**
     * @brief Check if predictor is initialized and ready
     */
    bool is_ready() const;

private:
    std::unique_ptr<TrajectoryPredictorImpl> impl_;
};

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * @brief Create route segments from a vector of waypoints
 * @param waypoints Vector of RouteWaypoint with cumulative distances set
 * @return Vector of RouteSegment
 */
inline std::vector<RouteSegment> create_route_from_waypoints(
    const std::vector<RouteWaypoint>& waypoints
) {
    std::vector<RouteSegment> segments;
    segments.reserve(waypoints.size() - 1);

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        double seg_dist = waypoints[i + 1].distance_from_origin_nm
                        - waypoints[i].distance_from_origin_nm;
        segments.emplace_back(waypoints[i], waypoints[i + 1], seg_dist);
    }

    return segments;
}

} // namespace openap
