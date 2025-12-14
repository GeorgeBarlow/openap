"""
Trajectory Prediction Example using OpenAP
==========================================

This example demonstrates how to use OpenAP for trajectory prediction
with a real flight route from LTFM (Istanbul) to LFPG (Paris CDG).

Aircraft: B738 (Boeing 737-800)
Cruise Altitude: FL370 (37,000 ft)
Route: LTFM BARPE GOLDO ALX IDILO SOSUS SUTIS DISOR ENFAR RETRA MADOS
       SIPAL BAXON SRN PEPAG ABESI UTAVO ELMUR RIPUS DITON HOC MOROK
       PENDU JAVVU GIVRI TINIL LFPG
"""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple
import numpy as np

# Import OpenAP components
from openap import FlightGenerator, WRAP
from openap.extra import aero


# =============================================================================
# DATA STRUCTURES
# =============================================================================

class FlightPhase(Enum):
    GROUND = "ground"
    CLIMB = "climb"
    CRUISE = "cruise"
    DESCENT = "descent"


@dataclass
class Position:
    latitude: float   # degrees
    longitude: float  # degrees

    def __repr__(self):
        lat_dir = "N" if self.latitude >= 0 else "S"
        lon_dir = "E" if self.longitude >= 0 else "W"
        return f"{abs(self.latitude):.4f}°{lat_dir}, {abs(self.longitude):.4f}°{lon_dir}"


@dataclass
class Waypoint:
    identifier: str
    position: Position
    distance_from_origin_nm: float = 0.0  # Cumulative distance from departure


@dataclass
class RouteSegment:
    from_wpt: Waypoint
    to_wpt: Waypoint
    distance_nm: float


@dataclass
class AircraftState:
    """Current aircraft state - mirrors your C++ struct"""
    position: Position
    pressure_altitude_ft: int
    ground_speed_kts: float
    track_heading: int
    vertical_speed_fpm: int
    is_on_ground: bool

    def get_phase(self, threshold_fpm: int = 300) -> FlightPhase:
        if self.is_on_ground:
            return FlightPhase.GROUND
        if self.vertical_speed_fpm > threshold_fpm:
            return FlightPhase.CLIMB
        if self.vertical_speed_fpm < -threshold_fpm:
            return FlightPhase.DESCENT
        return FlightPhase.CRUISE


@dataclass
class TrajectoryPrediction:
    """Complete trajectory prediction result"""
    # Time predictions (seconds from now)
    time_to_toc_sec: Optional[float]
    time_to_tod_sec: Optional[float]
    time_to_destination_sec: float

    # Distance predictions (nautical miles)
    distance_to_toc_nm: Optional[float]
    distance_to_tod_nm: Optional[float]
    distance_remaining_nm: float

    # Position predictions
    toc_position: Optional[Position]
    tod_position: Optional[Position]

    # Phase durations
    climb_duration_sec: float
    cruise_duration_sec: float
    descent_duration_sec: float

    # Altitudes
    cruise_altitude_ft: int

    # Current status
    current_phase: FlightPhase

    def __repr__(self):
        lines = [
            "=" * 70,
            "TRAJECTORY PREDICTION RESULTS",
            "=" * 70,
            f"Current Phase: {self.current_phase.value.upper()}",
            f"Distance Remaining: {self.distance_remaining_nm:.1f} NM",
            f"Cruise Altitude: FL{self.cruise_altitude_ft // 100}",
            "",
            "TIMING:",
        ]

        if self.time_to_toc_sec is not None:
            lines.append(f"  Time to TOC: {self._format_time(self.time_to_toc_sec)}")
        if self.time_to_tod_sec is not None:
            lines.append(f"  Time to TOD: {self._format_time(self.time_to_tod_sec)}")
        lines.append(f"  Time to Destination: {self._format_time(self.time_to_destination_sec)}")

        lines.extend([
            "",
            "PHASE DURATIONS:",
            f"  Climb: {self._format_time(self.climb_duration_sec)}",
            f"  Cruise: {self._format_time(self.cruise_duration_sec)}",
            f"  Descent: {self._format_time(self.descent_duration_sec)}",
        ])

        if self.toc_position:
            lines.extend([
                "",
                f"TOP OF CLIMB Position: {self.toc_position}",
                f"  Distance from origin: {self.distance_to_toc_nm:.1f} NM" if self.distance_to_toc_nm else "",
            ])

        if self.tod_position:
            lines.extend([
                "",
                f"TOP OF DESCENT Position: {self.tod_position}",
                f"  Distance from origin: {self.distance_to_tod_nm:.1f} NM" if self.distance_to_tod_nm else "",
            ])

        lines.append("=" * 70)
        return "\n".join(lines)

    @staticmethod
    def _format_time(seconds: float) -> str:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        if hours > 0:
            return f"{hours}h {minutes:02d}m {secs:02d}s"
        return f"{minutes}m {secs:02d}s"


# =============================================================================
# TRAJECTORY PREDICTOR CLASS
# =============================================================================

class TrajectoryPredictor:
    """
    Trajectory predictor using OpenAP kinematic models.
    """

    def __init__(self, aircraft_type: str):
        """
        Initialize predictor for a specific aircraft type.

        Args:
            aircraft_type: ICAO aircraft type code (e.g., "B738", "A320")
        """
        self.aircraft_type = aircraft_type.upper()
        self.flight_gen = FlightGenerator(ac=aircraft_type, use_synonym=True)
        self.wrap = WRAP(aircraft_type, use_synonym=True)

        # Cache performance defaults
        self._load_performance_data()

        print(f"Initialized TrajectoryPredictor for {self.aircraft_type}")
        print(f"  Default cruise Mach: {self.cruise_mach:.3f}")
        print(f"  Default cruise altitude: FL{int(self.cruise_alt_ft / 100)}")
        print(f"  Typical climb rate: {self.climb_vs_fpm:.0f} fpm")
        print(f"  Typical descent rate: {self.descent_vs_fpm:.0f} fpm")

    def _load_performance_data(self):
        """Load performance characteristics from WRAP model"""
        # Climb parameters
        self.climb_cas_kts = self.wrap.climb_const_vcas()["default"] / aero.kts
        self.climb_mach = self.wrap.climb_const_mach()["default"]
        self.climb_vs_fpm = self.wrap.climb_vs_concas()["default"] / aero.fpm

        # Cruise parameters
        self.cruise_mach = self.wrap.cruise_mach()["default"]
        self.cruise_alt_ft = self.wrap.cruise_alt()["default"] * 1000 / aero.ft

        # Descent parameters
        self.descent_cas_kts = self.wrap.descent_const_vcas()["default"] / aero.kts
        self.descent_mach = self.wrap.descent_const_mach()["default"]
        self.descent_vs_fpm = abs(self.wrap.descent_vs_concas()["default"] / aero.fpm)

    # -------------------------------------------------------------------------
    # Distance calculations
    # -------------------------------------------------------------------------

    def distance_nm(self, pos1: Position, pos2: Position) -> float:
        """Calculate great circle distance in nautical miles"""
        dist_m = aero.distance(pos1.latitude, pos1.longitude,
                               pos2.latitude, pos2.longitude)
        return dist_m / aero.nm

    def bearing_deg(self, pos1: Position, pos2: Position) -> float:
        """Calculate bearing from pos1 to pos2"""
        return aero.bearing(pos1.latitude, pos1.longitude,
                           pos2.latitude, pos2.longitude)

    def position_along_bearing(self, start: Position, bearing: float,
                                distance_nm: float) -> Position:
        """Calculate position at distance along bearing"""
        dist_m = distance_nm * aero.nm
        lat, lon = aero.latlon(start.latitude, start.longitude, dist_m, bearing)
        return Position(latitude=lat, longitude=lon)

    # -------------------------------------------------------------------------
    # Profile generation using OpenAP
    # -------------------------------------------------------------------------

    def generate_climb_profile(self, start_alt_ft: int, cruise_alt_ft: int,
                                dt: int = 10) -> dict:
        """
        Generate climb profile from start altitude to cruise.

        Returns:
            dict with duration_sec, distance_nm, and raw profile DataFrame
        """
        if start_alt_ft >= cruise_alt_ft:
            return {"duration_sec": 0, "distance_nm": 0, "profile": None}

        # Generate full climb from ground
        full_climb = self.flight_gen.climb(dt=dt, alt_cr=cruise_alt_ft)

        # Find where current altitude intersects the climb profile
        start_alt_m = start_alt_ft * aero.ft

        # Find first point at or above start altitude
        mask = full_climb['h'] >= start_alt_m
        if not mask.any():
            return {"duration_sec": 0, "distance_nm": 0, "profile": None}

        remaining = full_climb[mask].copy()

        # Adjust time and distance to start from 0
        t_offset = remaining['t'].iloc[0]
        s_offset = remaining['s'].iloc[0]
        remaining['t'] = remaining['t'] - t_offset
        remaining['s'] = remaining['s'] - s_offset

        return {
            "duration_sec": remaining['t'].iloc[-1],
            "distance_nm": remaining['s'].iloc[-1] / aero.nm,
            "profile": remaining
        }

    def generate_descent_profile(self, cruise_alt_ft: int, dt: int = 10) -> dict:
        """
        Generate descent profile from cruise to landing.

        Returns:
            dict with duration_sec, distance_nm, and raw profile DataFrame
        """
        descent = self.flight_gen.descent(
            dt=dt,
            alt_cr=cruise_alt_ft,
            withcr=False  # Don't include cruise segment
        )

        return {
            "duration_sec": descent['t'].iloc[-1],
            "distance_nm": descent['s'].iloc[-1] / aero.nm,
            "profile": descent
        }

    def cruise_speed_kts(self, altitude_ft: int) -> float:
        """Get cruise true airspeed in knots at given altitude"""
        alt_m = altitude_ft * aero.ft
        tas_ms = aero.mach2tas(self.cruise_mach, alt_m)
        return tas_ms / aero.kts

    # -------------------------------------------------------------------------
    # Route analysis
    # -------------------------------------------------------------------------

    def find_position_on_route(self, segments: List[RouteSegment],
                                distance_nm: float) -> Optional[Position]:
        """
        Find lat/lon position at given distance along route.
        """
        accumulated = 0.0

        for seg in segments:
            if accumulated + seg.distance_nm >= distance_nm:
                # Position is on this segment
                dist_into_seg = distance_nm - accumulated
                bearing = self.bearing_deg(seg.from_wpt.position,
                                           seg.to_wpt.position)
                return self.position_along_bearing(
                    seg.from_wpt.position, bearing, dist_into_seg
                )
            accumulated += seg.distance_nm

        # Past end of route - return destination
        if segments:
            return segments[-1].to_wpt.position
        return None

    def find_nearest_waypoint(self, segments: List[RouteSegment],
                               target_distance_nm: float) -> Tuple[str, float]:
        """
        Find nearest waypoint to a given distance along route.

        Returns:
            Tuple of (waypoint_identifier, offset_nm)
            Positive offset = after waypoint, negative = before
        """
        # Build list of waypoints with distances
        waypoints = [(segments[0].from_wpt.identifier, 0.0)]
        accumulated = 0.0

        for seg in segments:
            accumulated += seg.distance_nm
            waypoints.append((seg.to_wpt.identifier, accumulated))

        # Find nearest
        min_offset = float('inf')
        nearest_wpt = ""

        for wpt_id, wpt_dist in waypoints:
            offset = target_distance_nm - wpt_dist
            if abs(offset) < abs(min_offset):
                min_offset = offset
                nearest_wpt = wpt_id

        return nearest_wpt, min_offset

    def find_current_position_on_route(self, state: AircraftState,
                                        segments: List[RouteSegment]) -> Tuple[int, float, float]:
        """
        Find where aircraft is along the route.

        Returns:
            Tuple of (segment_index, distance_into_segment_nm, total_distance_from_origin_nm)
        """
        best_seg_idx = 0
        best_dist_along = 0.0
        min_cross_track = float('inf')

        for idx, seg in enumerate(segments):
            from_pos = seg.from_wpt.position
            to_pos = seg.to_wpt.position

            # Distance from 'from' waypoint to aircraft
            dist_to_from = self.distance_nm(from_pos, state.position)

            # Segment length
            seg_length = seg.distance_nm
            if seg_length < 0.1:
                continue

            # Bearings
            seg_bearing = self.bearing_deg(from_pos, to_pos)
            ac_bearing = self.bearing_deg(from_pos, state.position)

            # Along-track and cross-track distances
            bearing_diff = np.radians(ac_bearing - seg_bearing)
            along_track = dist_to_from * np.cos(bearing_diff)
            cross_track = abs(dist_to_from * np.sin(bearing_diff))

            # Check if aircraft is "on" this segment
            if 0 <= along_track <= seg_length and cross_track < min_cross_track:
                min_cross_track = cross_track
                best_seg_idx = idx
                best_dist_along = along_track

        # Calculate total distance from origin
        total_dist = sum(seg.distance_nm for seg in segments[:best_seg_idx]) + best_dist_along

        return best_seg_idx, best_dist_along, total_dist

    # -------------------------------------------------------------------------
    # Main prediction methods
    # -------------------------------------------------------------------------

    def predict_from_ground(self, segments: List[RouteSegment],
                             cruise_altitude_ft: int) -> TrajectoryPrediction:
        """
        Predict trajectory for aircraft on the ground (pre-departure).
        """
        # Total route distance
        total_dist_nm = sum(seg.distance_nm for seg in segments)

        # Generate climb profile (from ground to cruise)
        climb = self.generate_climb_profile(0, cruise_altitude_ft)
        climb_dist_nm = climb["distance_nm"]
        climb_time_sec = climb["duration_sec"]

        # Generate descent profile
        descent = self.generate_descent_profile(cruise_altitude_ft)
        descent_dist_nm = descent["distance_nm"]
        descent_time_sec = descent["duration_sec"]

        # Calculate cruise segment
        cruise_dist_nm = max(0, total_dist_nm - climb_dist_nm - descent_dist_nm)
        cruise_speed = self.cruise_speed_kts(cruise_altitude_ft)
        cruise_time_sec = (cruise_dist_nm / cruise_speed) * 3600 if cruise_speed > 0 else 0

        # Find TOC position
        toc_position = self.find_position_on_route(segments, climb_dist_nm)
        toc_wpt, toc_offset = self.find_nearest_waypoint(segments, climb_dist_nm)

        # Find TOD position
        tod_dist_from_origin = total_dist_nm - descent_dist_nm
        tod_position = self.find_position_on_route(segments, tod_dist_from_origin)
        tod_wpt, tod_offset = self.find_nearest_waypoint(segments, tod_dist_from_origin)

        print(f"\n--- Prediction Details (From Ground) ---")
        print(f"Total route distance: {total_dist_nm:.1f} NM")
        print(f"Climb distance: {climb_dist_nm:.1f} NM ({climb_time_sec/60:.1f} min)")
        print(f"Cruise distance: {cruise_dist_nm:.1f} NM ({cruise_time_sec/60:.1f} min)")
        print(f"Descent distance: {descent_dist_nm:.1f} NM ({descent_time_sec/60:.1f} min)")
        print(f"TOC near: {toc_wpt} ({toc_offset:+.1f} NM)")
        print(f"TOD near: {tod_wpt} ({tod_offset:+.1f} NM)")

        return TrajectoryPrediction(
            time_to_toc_sec=climb_time_sec,
            time_to_tod_sec=climb_time_sec + cruise_time_sec,
            time_to_destination_sec=climb_time_sec + cruise_time_sec + descent_time_sec,
            distance_to_toc_nm=climb_dist_nm,
            distance_to_tod_nm=tod_dist_from_origin,
            distance_remaining_nm=total_dist_nm,
            toc_position=toc_position,
            tod_position=tod_position,
            climb_duration_sec=climb_time_sec,
            cruise_duration_sec=cruise_time_sec,
            descent_duration_sec=descent_time_sec,
            cruise_altitude_ft=cruise_altitude_ft,
            current_phase=FlightPhase.GROUND
        )

    def predict_from_airborne(self, state: AircraftState,
                               segments: List[RouteSegment],
                               cruise_altitude_ft: int) -> TrajectoryPrediction:
        """
        Predict trajectory for aircraft currently in flight.
        """
        # Find current position along route
        seg_idx, dist_along, dist_from_origin = self.find_current_position_on_route(
            state, segments
        )

        # Calculate remaining distance
        total_dist_nm = sum(seg.distance_nm for seg in segments)
        remaining_dist_nm = total_dist_nm - dist_from_origin

        current_phase = state.get_phase()

        print(f"\n--- Prediction Details (Airborne) ---")
        print(f"Current phase: {current_phase.value}")
        print(f"Current altitude: {state.pressure_altitude_ft} ft")
        print(f"Distance flown: {dist_from_origin:.1f} NM")
        print(f"Distance remaining: {remaining_dist_nm:.1f} NM")

        if current_phase == FlightPhase.CLIMB:
            # Generate remaining climb
            climb = self.generate_climb_profile(state.pressure_altitude_ft, cruise_altitude_ft)
            remaining_climb_dist = climb["distance_nm"]
            remaining_climb_time = climb["duration_sec"]

            # Descent profile
            descent = self.generate_descent_profile(cruise_altitude_ft)
            descent_dist_nm = descent["distance_nm"]
            descent_time_sec = descent["duration_sec"]

            # Cruise
            cruise_dist_nm = max(0, remaining_dist_nm - remaining_climb_dist - descent_dist_nm)
            cruise_speed = self.cruise_speed_kts(cruise_altitude_ft)
            cruise_time_sec = (cruise_dist_nm / cruise_speed) * 3600 if cruise_speed > 0 else 0

            # TOC position (from current position)
            toc_dist_from_origin = dist_from_origin + remaining_climb_dist
            toc_position = self.find_position_on_route(segments, toc_dist_from_origin)

            # TOD position
            tod_dist_from_origin = total_dist_nm - descent_dist_nm
            tod_position = self.find_position_on_route(segments, tod_dist_from_origin)

            return TrajectoryPrediction(
                time_to_toc_sec=remaining_climb_time,
                time_to_tod_sec=remaining_climb_time + cruise_time_sec,
                time_to_destination_sec=remaining_climb_time + cruise_time_sec + descent_time_sec,
                distance_to_toc_nm=toc_dist_from_origin,
                distance_to_tod_nm=tod_dist_from_origin,
                distance_remaining_nm=remaining_dist_nm,
                toc_position=toc_position,
                tod_position=tod_position,
                climb_duration_sec=remaining_climb_time,
                cruise_duration_sec=cruise_time_sec,
                descent_duration_sec=descent_time_sec,
                cruise_altitude_ft=cruise_altitude_ft,
                current_phase=FlightPhase.CLIMB
            )

        elif current_phase == FlightPhase.CRUISE:
            # No remaining climb
            descent = self.generate_descent_profile(state.pressure_altitude_ft)
            descent_dist_nm = descent["distance_nm"]
            descent_time_sec = descent["duration_sec"]

            # Remaining cruise
            cruise_dist_nm = max(0, remaining_dist_nm - descent_dist_nm)
            cruise_speed = self.cruise_speed_kts(state.pressure_altitude_ft)
            cruise_time_sec = (cruise_dist_nm / cruise_speed) * 3600 if cruise_speed > 0 else 0

            # TOD position
            tod_dist_from_origin = total_dist_nm - descent_dist_nm
            tod_position = self.find_position_on_route(segments, tod_dist_from_origin)

            return TrajectoryPrediction(
                time_to_toc_sec=None,  # Already past TOC
                time_to_tod_sec=cruise_time_sec,
                time_to_destination_sec=cruise_time_sec + descent_time_sec,
                distance_to_toc_nm=None,
                distance_to_tod_nm=tod_dist_from_origin,
                distance_remaining_nm=remaining_dist_nm,
                toc_position=None,
                tod_position=tod_position,
                climb_duration_sec=0,
                cruise_duration_sec=cruise_time_sec,
                descent_duration_sec=descent_time_sec,
                cruise_altitude_ft=state.pressure_altitude_ft,
                current_phase=FlightPhase.CRUISE
            )

        else:  # DESCENT
            # Simple descent time calculation
            alt_remaining = state.pressure_altitude_ft
            descent_time_sec = (alt_remaining / self.descent_vs_fpm) * 60

            return TrajectoryPrediction(
                time_to_toc_sec=None,
                time_to_tod_sec=None,  # Already past TOD
                time_to_destination_sec=descent_time_sec,
                distance_to_toc_nm=None,
                distance_to_tod_nm=None,
                distance_remaining_nm=remaining_dist_nm,
                toc_position=None,
                tod_position=None,
                climb_duration_sec=0,
                cruise_duration_sec=0,
                descent_duration_sec=descent_time_sec,
                cruise_altitude_ft=0,
                current_phase=FlightPhase.DESCENT
            )

    def predict(self, state: AircraftState,
                segments: List[RouteSegment],
                cruise_altitude_ft: int) -> TrajectoryPrediction:
        """
        Main prediction entry point - handles both ground and airborne.
        """
        if state.is_on_ground:
            return self.predict_from_ground(segments, cruise_altitude_ft)
        else:
            return self.predict_from_airborne(state, segments, cruise_altitude_ft)


# =============================================================================
# EXAMPLE FLIGHT DATA: LTFM -> LFPG
# =============================================================================

def create_example_route() -> Tuple[List[Waypoint], List[RouteSegment]]:
    """
    Create the example route from LTFM to LFPG.

    Route: LTFM BARPE GOLDO ALX IDILO SOSUS SUTIS DISOR ENFAR RETRA MADOS
           SIPAL BAXON SRN PEPAG ABESI UTAVO ELMUR RIPUS DITON HOC MOROK
           PENDU JAVVU GIVRI TINIL LFPG
    """
    # Waypoint data from your example (identifier, lat, lon, cumulative_dist_nm)
    waypoint_data = [
        ("LTFM",  41.2753,  28.7519,    0.0),    # Istanbul Airport
        ("BARPE", 40.9261,  26.9781,   82.3),
        ("GOLDO", 40.8822,  26.2494,  115.5),
        ("ALX",   40.8550,  25.9572,  128.8),
        ("IDILO", 40.7906,  25.4394,  152.7),
        ("SOSUS", 40.7442,  25.0733,  169.6),
        ("SUTIS", 40.7019,  24.7486,  184.6),
        ("DISOR", 41.2472,  22.7583,  280.5),
        ("ENFAR", 41.7686,  20.5344,  385.3),
        ("RETRA", 42.2283,  19.3350,  445.5),
        ("MADOS", 42.6025,  18.2492,  498.6),
        ("SIPAL", 43.1367,  17.0736,  559.5),
        ("BAXON", 44.4164,  13.4631,  733.8),
        ("SRN",   45.6468,   9.0229,  936.2),
        ("PEPAG", 45.9839,   9.0714,  956.5),
        ("ABESI", 46.1597,   9.0428,  967.1),
        ("UTAVO", 46.4106,   9.0092,  982.3),
        ("ELMUR", 47.1568,   8.9076, 1027.3),
        ("RIPUS", 47.2603,   8.5000, 1045.0),
        ("DITON", 47.3022,   8.3333, 1052.3),
        ("HOC",   47.4666,   7.6654, 1081.1),
        ("MOROK", 47.3966,   6.6555, 1122.4),
        ("PENDU", 47.3489,   6.0326, 1147.9),
        ("JAVVU", 47.3323,   5.8322, 1156.1),
        ("GIVRI", 47.2919,   5.3422, 1176.2),
        ("TINIL", 47.5889,   5.0986, 1196.6),
        ("LFPG",  49.0128,   2.5500, 1329.5),   # Paris CDG
    ]

    # Create waypoint objects
    waypoints = []
    for ident, lat, lon, dist in waypoint_data:
        wpt = Waypoint(
            identifier=ident,
            position=Position(latitude=lat, longitude=lon),
            distance_from_origin_nm=dist
        )
        waypoints.append(wpt)

    # Create segments
    segments = []
    for i in range(len(waypoints) - 1):
        from_wpt = waypoints[i]
        to_wpt = waypoints[i + 1]
        seg_dist = to_wpt.distance_from_origin_nm - from_wpt.distance_from_origin_nm

        seg = RouteSegment(
            from_wpt=from_wpt,
            to_wpt=to_wpt,
            distance_nm=seg_dist
        )
        segments.append(seg)

    return waypoints, segments


# =============================================================================
# MAIN EXAMPLE
# =============================================================================

def main():
    print("=" * 70)
    print("OPENAP TRAJECTORY PREDICTION EXAMPLE")
    print("Flight: LTFM (Istanbul) -> LFPG (Paris CDG)")
    print("Aircraft: B738 (Boeing 737-800)")
    print("=" * 70)

    # Create route
    waypoints, segments = create_example_route()
    cruise_altitude_ft = 37000  # FL370

    print(f"\nRoute has {len(waypoints)} waypoints and {len(segments)} segments")
    print(f"Total distance: {sum(seg.distance_nm for seg in segments):.1f} NM")
    print(f"Cruise altitude: FL{cruise_altitude_ft // 100}")

    # Initialize predictor
    print("\n" + "-" * 70)
    predictor = TrajectoryPredictor("B738")

    # =========================================================================
    # SCENARIO 1: Aircraft on the ground at LTFM (pre-departure)
    # =========================================================================
    print("\n" + "=" * 70)
    print("SCENARIO 1: Aircraft on ground at LTFM (pre-departure)")
    print("=" * 70)

    ground_state = AircraftState(
        position=Position(latitude=41.2753, longitude=28.7519),  # LTFM
        pressure_altitude_ft=0,
        ground_speed_kts=0,
        track_heading=0,
        vertical_speed_fpm=0,
        is_on_ground=True
    )

    prediction = predictor.predict(ground_state, segments, cruise_altitude_ft)
    print(prediction)

    # =========================================================================
    # SCENARIO 2: Aircraft climbing through FL200 near BARPE
    # =========================================================================
    print("\n" + "=" * 70)
    print("SCENARIO 2: Aircraft climbing through FL200 near BARPE")
    print("=" * 70)

    climbing_state = AircraftState(
        position=Position(latitude=40.95, longitude=27.2),  # Between LTFM and BARPE
        pressure_altitude_ft=20000,
        ground_speed_kts=420,
        track_heading=280,
        vertical_speed_fpm=1800,  # Climbing
        is_on_ground=False
    )

    prediction = predictor.predict(climbing_state, segments, cruise_altitude_ft)
    print(prediction)

    # =========================================================================
    # SCENARIO 3: Aircraft in cruise at FL370 near BAXON (mid-flight)
    # =========================================================================
    print("\n" + "=" * 70)
    print("SCENARIO 3: Aircraft in cruise at FL370 near BAXON")
    print("=" * 70)

    cruise_state = AircraftState(
        position=Position(latitude=44.4164, longitude=13.4631),  # BAXON
        pressure_altitude_ft=37000,
        ground_speed_kts=453,
        track_heading=290,
        vertical_speed_fpm=0,  # Level flight
        is_on_ground=False
    )

    prediction = predictor.predict(cruise_state, segments, cruise_altitude_ft)
    print(prediction)

    # =========================================================================
    # SCENARIO 4: Aircraft descending through FL250 approaching LFPG
    # =========================================================================
    print("\n" + "=" * 70)
    print("SCENARIO 4: Aircraft descending through FL250 near TINIL")
    print("=" * 70)

    descending_state = AircraftState(
        position=Position(latitude=48.0, longitude=4.5),  # Past TINIL
        pressure_altitude_ft=25000,
        ground_speed_kts=380,
        track_heading=320,
        vertical_speed_fpm=-1500,  # Descending
        is_on_ground=False
    )

    prediction = predictor.predict(descending_state, segments, cruise_altitude_ft)
    print(prediction)

    # =========================================================================
    # Show raw OpenAP profile data
    # =========================================================================
    print("\n" + "=" * 70)
    print("RAW OPENAP CLIMB PROFILE (first 20 rows)")
    print("=" * 70)

    climb_profile = predictor.flight_gen.climb(dt=30, alt_cr=cruise_altitude_ft)
    print("\nColumns: t(sec), h(m), s(m), v(m/s), vs(m/s), seg(phase)")
    print(climb_profile[['t', 'h', 's', 'v', 'vs', 'seg', 'altitude', 'groundspeed']].head(20).to_string())

    print("\n" + "=" * 70)
    print("RAW OPENAP DESCENT PROFILE (first 20 rows)")
    print("=" * 70)

    descent_profile = predictor.flight_gen.descent(dt=30, alt_cr=cruise_altitude_ft, withcr=False)
    print("\nColumns: t(sec), h(m), s(m), v(m/s), vs(m/s), seg(phase)")
    print(descent_profile[['t', 'h', 's', 'v', 'vs', 'seg', 'altitude', 'groundspeed']].head(20).to_string())

    # =========================================================================
    # Performance characteristics
    # =========================================================================
    print("\n" + "=" * 70)
    print("B738 PERFORMANCE CHARACTERISTICS FROM OPENAP WRAP MODEL")
    print("=" * 70)

    wrap = predictor.wrap

    print("\nCLIMB:")
    print(f"  Constant CAS: {wrap.climb_const_vcas()['default']/aero.kts:.0f} kt")
    print(f"  Constant Mach: {wrap.climb_const_mach()['default']:.3f}")
    print(f"  CAS climb V/S: {wrap.climb_vs_concas()['default']/aero.fpm:.0f} fpm")
    print(f"  Mach climb V/S: {wrap.climb_vs_conmach()['default']/aero.fpm:.0f} fpm")

    print("\nCRUISE:")
    print(f"  Typical Mach: {wrap.cruise_mach()['default']:.3f}")
    print(f"  Typical altitude: {wrap.cruise_alt()['default']*1000/aero.ft:.0f} ft")

    print("\nDESCENT:")
    print(f"  Constant CAS: {wrap.descent_const_vcas()['default']/aero.kts:.0f} kt")
    print(f"  Constant Mach: {wrap.descent_const_mach()['default']:.3f}")
    print(f"  CAS descent V/S: {wrap.descent_vs_concas()['default']/aero.fpm:.0f} fpm")
    print(f"  Final approach V/S: {wrap.finalapp_vs()['default']/aero.fpm:.0f} fpm")


if __name__ == "__main__":
    main()
