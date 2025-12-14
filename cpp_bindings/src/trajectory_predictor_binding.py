"""
trajectory_predictor_binding.py
================================

Python binding module for C++ interop via pybind11.
This module provides a wrapper class that the C++ code calls.
"""

import sys
import os

# Add the openap examples directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
examples_dir = os.path.join(script_dir, '..', '..', 'examples')
if examples_dir not in sys.path:
    sys.path.insert(0, examples_dir)

# Import the trajectory predictor from the examples
from trajectory_prediction_example import (
    TrajectoryPredictor,
    TrajectoryPrediction,
    AircraftState,
    Position,
    RouteWaypoint,
    RouteSegment,
    FlightPhase
)


class TrajectoryPredictorWrapper:
    """
    Wrapper class for C++ interop.

    This class provides a simplified interface that accepts dictionaries
    (which pybind11 converts from C++ objects) and returns results that
    can be easily converted back to C++ structs.
    """

    def __init__(self, aircraft_type: str):
        """
        Initialize the predictor for a specific aircraft type.

        Args:
            aircraft_type: ICAO aircraft type code (e.g., "B738", "A320")
        """
        self.predictor = TrajectoryPredictor(aircraft_type)
        self._aircraft_type = aircraft_type

    def predict(self, state_dict: dict, segments_list: list,
                cruise_altitude_ft: int) -> TrajectoryPrediction:
        """
        Generate trajectory prediction from C++ data.

        Args:
            state_dict: Dictionary with aircraft state fields
            segments_list: List of segment dictionaries
            cruise_altitude_ft: Target cruise altitude

        Returns:
            TrajectoryPrediction object
        """
        # Convert state dictionary to AircraftState object
        state = AircraftState(
            position=Position(
                latitude=state_dict['latitude'],
                longitude=state_dict['longitude']
            ),
            pressure_altitude_ft=state_dict['pressure_altitude_ft'],
            ground_speed_kts=state_dict['ground_speed_kts'],
            track_heading=state_dict['track_heading'],
            vertical_speed_fpm=state_dict['vertical_speed_fpm'],
            is_on_ground=state_dict['is_on_ground']
        )

        # Convert segment dictionaries to RouteSegment objects
        segments = []
        for seg_dict in segments_list:
            from_dict = seg_dict['from']
            to_dict = seg_dict['to']

            from_wpt = RouteWaypoint(
                identifier=from_dict['identifier'],
                position=Position(
                    latitude=from_dict['latitude'],
                    longitude=from_dict['longitude']
                ),
                distance_from_origin_nm=from_dict['distance_from_origin_nm']
            )

            to_wpt = RouteWaypoint(
                identifier=to_dict['identifier'],
                position=Position(
                    latitude=to_dict['latitude'],
                    longitude=to_dict['longitude']
                ),
                distance_from_origin_nm=to_dict['distance_from_origin_nm']
            )

            segment = RouteSegment(
                from_wpt=from_wpt,
                to_wpt=to_wpt,
                distance_nm=seg_dict['distance_nm']
            )
            segments.append(segment)

        # Call the predictor
        return self.predictor.predict(state, segments, cruise_altitude_ft)

    def get_performance_summary(self) -> dict:
        """
        Get aircraft performance characteristics.

        Returns:
            Dictionary with climb, cruise, descent parameters
        """
        return {
            'aircraft_type': self._aircraft_type,
            'climb': {
                'cas_kts': self.predictor.climb_cas_kts,
                'mach': self.predictor.climb_mach,
                'vs_fpm': self.predictor.climb_vs_fpm
            },
            'cruise': {
                'mach': self.predictor.cruise_mach,
                'altitude_ft': self.predictor.cruise_alt_ft
            },
            'descent': {
                'cas_kts': self.predictor.descent_cas_kts,
                'mach': self.predictor.descent_mach,
                'vs_fpm': self.predictor.descent_vs_fpm
            }
        }


# For testing the binding module directly
if __name__ == "__main__":
    print("Testing TrajectoryPredictorWrapper...")

    wrapper = TrajectoryPredictorWrapper("B738")

    # Test state
    state = {
        'latitude': 41.2753,
        'longitude': 28.7519,
        'pressure_altitude_ft': 0,
        'ground_speed_kts': 0,
        'track_heading': 0,
        'vertical_speed_fpm': 0,
        'is_on_ground': True
    }

    # Test segments (abbreviated route)
    segments = [
        {
            'from': {'identifier': 'LTFM', 'latitude': 41.2753, 'longitude': 28.7519, 'distance_from_origin_nm': 0.0},
            'to': {'identifier': 'BARPE', 'latitude': 40.9261, 'longitude': 26.9781, 'distance_from_origin_nm': 82.3},
            'distance_nm': 82.3,
            'airway': 'DCT'
        },
        {
            'from': {'identifier': 'BARPE', 'latitude': 40.9261, 'longitude': 26.9781, 'distance_from_origin_nm': 82.3},
            'to': {'identifier': 'GOLDO', 'latitude': 40.8822, 'longitude': 26.2494, 'distance_from_origin_nm': 115.5},
            'distance_nm': 33.2,
            'airway': 'DCT'
        },
    ]

    prediction = wrapper.predict(state, segments, 37000)
    print(f"Time to destination: {prediction.time_to_destination_sec:.0f} seconds")
    print(f"Current phase: {prediction.current_phase.value}")

    perf = wrapper.get_performance_summary()
    print(f"Cruise Mach: {perf['cruise']['mach']}")

    print("Test passed!")
