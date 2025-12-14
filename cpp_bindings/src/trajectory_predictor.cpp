/**
 * @file trajectory_predictor.cpp
 * @brief Implementation of TrajectoryPredictor using pybind11
 */

#include "trajectory_predictor.hpp"

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <mutex>
#include <iostream>

namespace py = pybind11;

namespace openap {

// =============================================================================
// Python Interpreter Management
// =============================================================================

namespace {

// Global Python interpreter guard
std::once_flag python_init_flag;
bool python_initialized = false;

void ensure_python_initialized() {
    std::call_once(python_init_flag, []() {
        if (!Py_IsInitialized()) {
            py::initialize_interpreter();
            python_initialized = true;
        }
    });
}

// Note: We don't finalize Python in this design because:
// 1. Multiple TrajectoryPredictor instances may exist
// 2. Python finalization can cause issues with static destruction order
// If needed, call py::finalize_interpreter() explicitly at program end

} // anonymous namespace

// =============================================================================
// TrajectoryPredictorImpl - PIMPL implementation
// =============================================================================

class TrajectoryPredictorImpl {
public:
    explicit TrajectoryPredictorImpl(const std::string& aircraft_type)
        : aircraft_type_(aircraft_type)
        , ready_(false)
    {
        ensure_python_initialized();
        initialize_predictor();
    }

    ~TrajectoryPredictorImpl() = default;

    TrajectoryPrediction predict(
        const AircraftState& state,
        const std::vector<RouteSegment>& route,
        int cruise_altitude_ft
    ) {
        py::gil_scoped_acquire acquire;

        try {
            // Convert AircraftState to Python dict
            py::dict py_state;
            py_state["latitude"] = state.position.latitude;
            py_state["longitude"] = state.position.longitude;
            py_state["pressure_altitude_ft"] = state.pressure_altitude_ft;
            py_state["ground_speed_kts"] = state.ground_speed_kts;
            py_state["track_heading"] = state.track_heading;
            py_state["reported_heading"] = state.reported_heading;
            py_state["vertical_speed_fpm"] = state.vertical_speed_fpm;
            py_state["is_on_ground"] = state.is_on_ground;

            // Convert route segments to Python list
            py::list py_segments;
            for (const auto& seg : route) {
                py::dict py_seg;

                py::dict from_wpt;
                from_wpt["identifier"] = seg.from.identifier;
                from_wpt["latitude"] = seg.from.position.latitude;
                from_wpt["longitude"] = seg.from.position.longitude;
                from_wpt["distance_from_origin_nm"] = seg.from.distance_from_origin_nm;
                py_seg["from"] = from_wpt;

                py::dict to_wpt;
                to_wpt["identifier"] = seg.to.identifier;
                to_wpt["latitude"] = seg.to.position.latitude;
                to_wpt["longitude"] = seg.to.position.longitude;
                to_wpt["distance_from_origin_nm"] = seg.to.distance_from_origin_nm;
                py_seg["to"] = to_wpt;

                py_seg["distance_nm"] = seg.distance_nm;
                py_seg["airway"] = seg.airway;

                py_segments.append(py_seg);
            }

            // Call Python predictor
            py::object result = predict_func_(py_state, py_segments, cruise_altitude_ft);

            // Convert result back to C++ struct
            return convert_prediction(result);

        } catch (const py::error_already_set& e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }

    AircraftPerformance get_performance() const {
        py::gil_scoped_acquire acquire;

        AircraftPerformance perf;
        perf.aircraft_type = aircraft_type_;

        try {
            py::object perf_dict = predictor_.attr("get_performance_summary")();

            py::dict climb = perf_dict["climb"].cast<py::dict>();
            perf.climb_cas_kts = climb["cas_kts"].cast<double>();
            perf.climb_mach = climb["mach"].cast<double>();
            perf.climb_vs_fpm = climb["vs_fpm"].cast<double>();

            py::dict cruise = perf_dict["cruise"].cast<py::dict>();
            perf.cruise_mach = cruise["mach"].cast<double>();
            perf.cruise_alt_ft = cruise["altitude_ft"].cast<double>();

            py::dict descent = perf_dict["descent"].cast<py::dict>();
            perf.descent_cas_kts = descent["cas_kts"].cast<double>();
            perf.descent_mach = descent["mach"].cast<double>();
            perf.descent_vs_fpm = descent["vs_fpm"].cast<double>();

        } catch (const py::error_already_set& e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }

        return perf;
    }

    const std::string& aircraft_type() const { return aircraft_type_; }
    bool is_ready() const { return ready_; }

private:
    void initialize_predictor() {
        py::gil_scoped_acquire acquire;

        try {
            // Import our trajectory predictor module
            // First, add the openap examples directory to sys.path
            py::module sys = py::module::import("sys");
            py::list path = sys.attr("path").cast<py::list>();

            // Add paths where trajectory_predictor.py might be
            // Adjust these paths based on your installation
            path.append(".");
            path.append("./examples");
            path.append("../examples");

            // Import the predictor module
            predictor_module_ = py::module::import("trajectory_predictor_binding");

            // Create predictor instance
            predictor_ = predictor_module_.attr("TrajectoryPredictorWrapper")(aircraft_type_);

            // Get the predict function
            predict_func_ = predictor_.attr("predict");

            ready_ = true;

        } catch (const py::error_already_set& e) {
            throw std::runtime_error(
                std::string("Failed to initialize Python predictor: ") + e.what()
            );
        }
    }

    TrajectoryPrediction convert_prediction(py::object& result) {
        TrajectoryPrediction pred;

        // Time predictions
        if (!result.attr("time_to_toc_sec").is_none()) {
            pred.time_to_toc_sec = result.attr("time_to_toc_sec").cast<double>();
        }
        if (!result.attr("time_to_tod_sec").is_none()) {
            pred.time_to_tod_sec = result.attr("time_to_tod_sec").cast<double>();
        }
        pred.time_to_destination_sec = result.attr("time_to_destination_sec").cast<double>();

        // Distance predictions
        if (!result.attr("distance_to_toc_nm").is_none()) {
            pred.distance_to_toc_nm = result.attr("distance_to_toc_nm").cast<double>();
        }
        if (!result.attr("distance_to_tod_nm").is_none()) {
            pred.distance_to_tod_nm = result.attr("distance_to_tod_nm").cast<double>();
        }
        pred.distance_remaining_nm = result.attr("distance_remaining_nm").cast<double>();

        // Position predictions
        if (!result.attr("toc_position").is_none()) {
            py::object toc = result.attr("toc_position");
            pred.toc_position = Position(
                toc.attr("latitude").cast<double>(),
                toc.attr("longitude").cast<double>()
            );
        }
        if (!result.attr("tod_position").is_none()) {
            py::object tod = result.attr("tod_position");
            pred.tod_position = Position(
                tod.attr("latitude").cast<double>(),
                tod.attr("longitude").cast<double>()
            );
        }

        // Phase durations
        pred.climb_duration_sec = result.attr("climb_duration_sec").cast<double>();
        pred.cruise_duration_sec = result.attr("cruise_duration_sec").cast<double>();
        pred.descent_duration_sec = result.attr("descent_duration_sec").cast<double>();

        // Altitude
        pred.cruise_altitude_ft = result.attr("cruise_altitude_ft").cast<int>();

        // Current phase
        std::string phase_str = result.attr("current_phase").attr("value").cast<std::string>();
        pred.current_phase = phase_from_string(phase_str);

        return pred;
    }

    std::string aircraft_type_;
    bool ready_;
    py::object predictor_module_;
    py::object predictor_;
    py::object predict_func_;
};

// =============================================================================
// TrajectoryPredictor Implementation
// =============================================================================

TrajectoryPredictor::TrajectoryPredictor(const std::string& aircraft_type)
    : impl_(std::make_unique<TrajectoryPredictorImpl>(aircraft_type))
{
}

TrajectoryPredictor::~TrajectoryPredictor() = default;

TrajectoryPredictor::TrajectoryPredictor(TrajectoryPredictor&&) noexcept = default;
TrajectoryPredictor& TrajectoryPredictor::operator=(TrajectoryPredictor&&) noexcept = default;

TrajectoryPrediction TrajectoryPredictor::predict(
    const AircraftState& state,
    const std::vector<RouteSegment>& route,
    int cruise_altitude_ft
) {
    return impl_->predict(state, route, cruise_altitude_ft);
}

AircraftPerformance TrajectoryPredictor::get_performance() const {
    return impl_->get_performance();
}

const std::string& TrajectoryPredictor::aircraft_type() const {
    return impl_->aircraft_type();
}

bool TrajectoryPredictor::is_ready() const {
    return impl_->is_ready();
}

} // namespace openap
