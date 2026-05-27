/**
 * Flight path engine — generates highly realistic, physically modeled 3D flight paths.
 * Encompasses departure (gate, taxi, hold, takeoff, climb) → cruise → arrival (descent, landing, rollout, taxi, gate).
 *
 * The engine calculates dynamic trajectories using:
 * 1. Altitude-dependent speed profiles (AGL — Above Ground Level)
 * 2. Numerical integration (Riemann sums) for variable-speed segments (climb & landing approach)
 * 3. Geodesic great-circle interpolation (using Cesium.EllipsoidGeodesic) for planetary cruising
 * 4. Micro-drift coordinate injection during holds to stabilize 3D model orientation vectors
 * 5. Quadratic constant-acceleration kinetics for takeoff rolls and linear rollout decelerations
 *
 * Speed profile (AGL — above ground level):
 *   Ground level: max 50 km/h (for taxiing & final rollout stages)
 *   0–400m AGL:   300 km/h (liftoff safety speed / final landing approach)
 *   400–800m AGL: 300 → 450 km/h (initial climb speed transition)
 *   800–3000m:    450 → 900 km/h (main acceleration ramp up)
 *   3000m+ AGL:   900 km/h (maximum commercial cruise speed, never exceeded)
 *
 * Climb Alignment:
 *   The climb phase goes completely STRAIGHT along the departure runway heading (no turns allowed)
 *   until reaching 3000m AGL, which mirrors realistic instrument departure (SID) procedures.
 * Landing Alignment:
 *   The final approach mirrors the climb speed profile in reverse, aligning the aircraft straight
 *   along the destination runway heading from 3000m AGL down to touchdown.
 */
import * as Cesium from "cesium";
import type { Airport, DepartureRoute, ArrivalRoute, RouteWaypoint } from "./airports";
import { getDistanceKm, DEPARTURE_ROUTES, getArrivalRoute } from "./airports";

export type FlightPhase =
  | "preflight"
  | "gate"          // Parked at gate, camera at front of plane
  | "taxi"          // Taxiing to runway
  | "runway_hold"   // Holding at runway threshold ("cleared for takeoff")
  | "takeoff"       // Accelerating on runway + liftoff
  | "climb"         // Climbing to cruise altitude
  | "climb_align"   // Smooth transition turn to align with cruise path
  | "cruise"        // Great-circle at cruise altitude
  | "descent_align" // Smooth transition turn to align with destination runway
  | "descent"       // Descending to approach altitude
  | "landing"       // Final approach + touchdown
  | "rollout"       // Decelerating on runway after touchdown
  | "arrival_taxi"  // Taxiing from runway to gate
  | "arrival_gate"  // Parked at destination gate
  | "arrived";      // Flight complete

export interface FlightTiming {
  gate: number;         // 0 if no departure route
  taxi: number;         // 0 if no departure route
  runwayHold: number;   // 0 if no departure route
  takeoff: number;      // runway roll (route) or generic takeoff (simple)
  climb: number;
  climbAlign: number;   // alignment turn between climb and cruise
  cruise: number;
  descentAlign: number; // alignment turn between cruise and descent
  descent: number;
  landing: number;
  rollout: number;      // 0 if no arrival route
  arrivalTaxi: number;  // 0 if no arrival route
  arrivalGate: number;  // 0 if no arrival route
}

export interface FlightPlan {
  departure: Airport;
  destination: Airport;
  distanceKm: number;
  departureRoute: DepartureRoute | null;
  arrivalRoute: ArrivalRoute | null;
  timing: FlightTiming;
  totalDuration: number;
}

export interface FlightResult {
  entity: Cesium.Entity;
  plan: FlightPlan;
  startTime: Cesium.JulianDate;
  stopTime: Cesium.JulianDate;
  getPhase: (currentTime: Cesium.JulianDate) => FlightPhase;
  /** If departure has a route, provides gate info for camera choreography */
  departureCamera: {
    gatePosition: RouteWaypoint;
    initialHeading: number; // radians — bearing from gate toward first taxi point
  } | null;
}

// ─── Speed & Altitude Constants ─────────────────────────────

const CRUISE_ALTITUDE = 10000;     // meters MSL (~33,000 ft)

// Speed constants (km/h) — exported for UI
export const MAX_SPEED_KMH = 900;
export const MAX_GROUND_SPEED_KMH = 50;
export const LIFTOFF_SPEED_KMH = 300;
export const CLIMB_MID_SPEED_KMH = 450;

// Speed constants (m/s)
const MAX_SPEED_MS = MAX_SPEED_KMH / 3.6;            
const MAX_GROUND_SPEED_MS = MAX_GROUND_SPEED_KMH / 3.6;  
const LIFTOFF_SPEED_MS = LIFTOFF_SPEED_KMH / 3.6;    
const CLIMB_MID_SPEED_MS = CLIMB_MID_SPEED_KMH / 3.6;   

// Altitude thresholds (AGL — meters above ground level)
const SPEED_RAMP_START_AGL = 400;    // Below: liftoff speed (300 km/h)
const SPEED_RAMP_MID_AGL = 800;      // 450 km/h reached here
const SPEED_RAMP_END_AGL = 3000;     // Full cruise speed (900 km/h) reached

// Geometry
const CLIMB_DISTANCE_DEG = 0.1;      // ~11 km horizontal climb distance
const APPROACH_DISTANCE_DEG = 0.25;  // ~27 km approach distance

// Fixed durations
const ARRIVAL_GATE_HOLD = 5;         // seconds parked at destination gate
const MIN_CRUISE_DURATION = 15;      // seconds minimum cruise time

// ─── Speed Profile ──────────────────────────────────────────

/**
 * Returns the airplane speed (m/s) based on altitude above ground level (AGL).
 * This models real-world airline operations where speed increases with altitude
 * to balance fuel efficiency, engine performance, and aerodynamic safety margins.
 *
 * Profile thresholds (AGL = altitude - ground elevation):
 *   AGL <= 0:            50 km/h   (Taxiing speed / safe ground maneuver)
 *   0 to 400m AGL:       300 km/h  (Initial climb safety speed / approach speed)
 *   400m to 800m AGL:    300 → 450 km/h (Linear speed ramp-up phase 1)
 *   800m to 3000m AGL:   450 → 900 km/h (Linear speed ramp-up phase 2)
 *   Above 3000m AGL:     900 km/h  (Maximum cruise speed)
 *
 * @param altitude Absolute aircraft altitude (Mean Sea Level - MSL) in meters.
 * @param groundElev The runway/airport elevation (MSL) in meters.
 * @returns Target velocity in meters per second (m/s).
 */
function getSpeedForAltitude(altitude: number, groundElev: number): number {
  const agl = altitude - groundElev; // Calculate height above ground level (AGL)
  if (agl <= 0) return MAX_GROUND_SPEED_MS; // Aircraft on ground -> taxi speed
  if (agl <= SPEED_RAMP_START_AGL) return LIFTOFF_SPEED_MS; // Low altitude flight -> safety buffer speed
  if (agl <= SPEED_RAMP_MID_AGL) {
    // Intermediate climb: linearly interpolate (LERP) speed from 300 to 450 km/h
    const frac = (agl - SPEED_RAMP_START_AGL) / (SPEED_RAMP_MID_AGL - SPEED_RAMP_START_AGL);
    return lerp(LIFTOFF_SPEED_MS, CLIMB_MID_SPEED_MS, frac);
  }
  if (agl <= SPEED_RAMP_END_AGL) {
    // Principal acceleration: linearly interpolate speed from 450 to 900 km/h
    const frac = (agl - SPEED_RAMP_MID_AGL) / (SPEED_RAMP_END_AGL - SPEED_RAMP_MID_AGL);
    return lerp(CLIMB_MID_SPEED_MS, MAX_SPEED_MS, frac);
  }
  return MAX_SPEED_MS; // Cruising altitude -> maximum permitted cruise velocity
}

//Distance Helpers

/** Haversine distance between two waypoints in meters. */
function waypointDistMeters(a: RouteWaypoint, b: RouteWaypoint): number {
  const R = 6371000;
  const dLat = (b.lat - a.lat) * Math.PI / 180;
  const dLon = (b.lon - a.lon) * Math.PI / 180;
  const lat1 = a.lat * Math.PI / 180;
  const lat2 = b.lat * Math.PI / 180;
  const sinDLat2 = Math.sin(dLat / 2);
  const sinDLon2 = Math.sin(dLon / 2);
  const h = sinDLat2 * sinDLat2 + Math.cos(lat1) * Math.cos(lat2) * sinDLon2 * sinDLon2;
  return R * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}

/** Total path length through an array of waypoints (meters). */
function computePathDistance(waypoints: RouteWaypoint[]): number {
  let total = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    total += waypointDistMeters(waypoints[i], waypoints[i + 1]);
  }
  return total;
}

/** Haversine distance between two lat/lon pairs in meters. */
function haversineDistMeters(lat1: number, lon1: number, lat2: number, lon2: number): number {
  return waypointDistMeters({ lat: lat1, lon: lon1 }, { lat: lat2, lon: lon2 });
}

// ─── Variable-Speed Timing (Numerical Integration) ─────────────────────────

/**
 * Calculates the total time (seconds) required to traverse a segment with altitude-dependent speeds.
 * Since the aircraft's speed varies continuously with its height (AGL) and the height changes as it climbs
 * or descends, a simple t = s/v formula is mathematically invalid.
 *
 * To resolve this, we employ NUMERICAL INTEGRATION (Riemann Sums):
 * 1. We partition the path into small micro-segments (e.g. 100 intervals).
 * 2. For each micro-segment, we interpolate the starting and ending heights to determine the vertical displacement.
 * 3. We calculate the true 3D straight-line distance of the segment using the Pythagorean theorem:
 *    segDist = sqrt(horizontal_distance^2 + vertical_altitude_change^2).
 * 4. We query the physical speed profile at the midpoint altitude of the segment.
 * 5. We calculate the micro-time elapsed (dt = segDist / speed) and accumulate it into totalTime.
 *
 * @param horizDistM Horizontal geodesic distance in meters.
 * @param startAlt Starting altitude (MSL) in meters.
 * @param endAlt Target ending altitude (MSL) in meters.
 * @param groundElev Terrain elevation (MSL) in meters.
 * @param segments Number of discrete integration intervals (higher number = greater precision).
 * @param easeFn Easing function to interpolate the altitude profile curve (e.g. easeOutQuad for landing).
 * @returns Total duration in seconds.
 */
function computeVariableSpeedDuration(
  horizDistM: number,
  startAlt: number,
  endAlt: number,
  groundElev: number,
  segments: number = 100,
  easeFn: (t: number) => number = (t) => t
): number {
  let totalTime = 0;
  for (let i = 0; i < segments; i++) {
    // Fraction of progress along the path at the start and end of this segment
    const f0 = i / segments;
    const f1 = (i + 1) / segments;
    const fMid = (f0 + f1) / 2; // Midpoint progress to evaluate average segment speed

    // Eased altitude at starting, ending, and middle points
    const alt0 = lerp(startAlt, endAlt, easeFn(f0));
    const alt1 = lerp(startAlt, endAlt, easeFn(f1));
    const altMid = lerp(startAlt, endAlt, easeFn(fMid));

    const segHorizDist = horizDistM / segments; // Horizontal length of the micro-segment
    const altChange = alt1 - alt0; // Vertical altitude gain/loss
    
    // True 3D straight-line distance in Euclidean space
    const segDist = Math.sqrt(segHorizDist * segHorizDist + altChange * altChange);

    // Retrieve the physical speed target for this midpoint altitude
    const speed = getSpeedForAltitude(altMid, groundElev);
    
    // Accumulate time step dt = ds / v
    totalTime += segDist / speed;
  }
  return totalTime;
}

/**
 * Computes the cumulative elapsed time (seconds) at each of the `samples+1` evenly-spaced path positions.
 * Returns an array of size `samples+1` where cumTimes[0] = 0 and cumTimes[samples] is the total time.
 * This cumulative time array maps directly to progress percentages, allowing us to assign exact
 * time offsets to Cesium's sampled position property for smooth, frame-rate independent rendering.
 *
 * @param horizDistM Total horizontal distance in meters.
 * @param startAlt Starting altitude (MSL) in meters.
 * @param endAlt Target ending altitude (MSL) in meters.
 * @param groundElev Terrain elevation (MSL) in meters.
 * @param samples Number of coordinate samples along the trajectory.
 * @param easeFn Altitude profile curve easing function.
 * @returns Array of cumulative seconds at each sample division.
 */
function computeCumulativeTimes(
  horizDistM: number,
  startAlt: number,
  endAlt: number,
  groundElev: number,
  samples: number,
  easeFn: (t: number) => number = (t) => t
): number[] {
  const cumTimes: number[] = [0];
  for (let i = 1; i <= samples; i++) {
    const f0 = (i - 1) / samples;
    const f1 = i / samples;
    const fMid = (f0 + f1) / 2;

    const alt0 = lerp(startAlt, endAlt, easeFn(f0));
    const alt1 = lerp(startAlt, endAlt, easeFn(f1));
    const altMid = lerp(startAlt, endAlt, easeFn(fMid));

    const segHorizDist = horizDistM / samples;
    const altChange = alt1 - alt0;
    
    // True 3D segment length calculation
    const segDist = Math.sqrt(segHorizDist * segHorizDist + altChange * altChange);

    // Determine local speed at the segment midpoint
    const speed = getSpeedForAltitude(altMid, groundElev);
    
    // Append the cumulative time step (dt = ds / v) to the array
    cumTimes.push(cumTimes[i - 1] + segDist / speed);
  }
  return cumTimes;
}

/**
 * For a linear speed ramp (v0 → v1) over a distance, compute the fraction
 * of total time elapsed at a given fraction of total distance.
 *
 * Uses analytical integral: timeFrac = ln(v(f)/v0) / ln(v1/v0).
 */
function linearSpeedTimeFrac(distFrac: number, v0: number, v1: number): number {
  if (Math.abs(v1 - v0) < 0.01) return distFrac; // constant speed
  const vAtF = v0 + (v1 - v0) * distFrac;
  return Math.log(vAtF / v0) / Math.log(v1 / v0);
}

/**
 * Total time to traverse a distance with linearly-changing speed v0 → v1.
 *
 * Analytical: T = D · ln(v1/v0) / (v1 − v0).
 */
function linearSpeedTotalTime(dist: number, v0: number, v1: number): number {
  if (Math.abs(v1 - v0) < 0.01) return dist / v0;
  return dist * Math.log(v1 / v0) / (v1 - v0);
}

// ─── Flight Planning & Geometry Setup (Plan Flight) ────────────────────────────

/**
 * Calculates the complete flight schedule and timing plan (used for the flight summary UI).
 * It computes the exact duration of each flight phase by analyzing GPS waypoints and applying
 * acceleration, deceleration, and altitude-dependent velocity integrals.
 *
 * Detailed execution flow:
 * 1. Fetches departure & arrival routing data for the designated airport codes.
 * 2. Sets fixed gate & hold times based on airport traffic regulations.
 * 3. Taxi: Integrates taxi path distances and divides by the ground speed limit (50 km/h).
 * 4. Takeoff: Uses constant acceleration kinematics (0 to 300 km/h). The average takeoff speed is
 *    v_avg = v_final / 2. Therefore, duration is exactly: t = 2 · runway_length / v_final.
 * 5. Climb: Computes runway heading from the final runway waypoints. Projects a climb exit point
 *    straight ahead by CLIMB_DISTANCE_DEG (~11 km). Uses numerical integration to calculate climb
 *    duration from runway elevation + 50m up to 3000m AGL.
 * 6. Approach/Landing: Identifies the runway threshold at the destination, calculates the landing heading,
 *    and projects an approach entry point backward along that heading by APPROACH_DISTANCE_DEG (~27 km).
 *    Computes duration from 3000m AGL down to runway elevation using an eased altitude profile (easeOutQuad)
 *    to represent the gradual roundout (flare) maneuver.
 * 7. Cruise & Descent: Connects the climb exit and approach entry points using a Great Circle path.
 *    Splits this geodesic route: 80% is allocated to cruising at 10,000m, and 20% to descent.
 *    Cruise and descent times are computed using the maximum cruising speed of 900 km/h (since the plane
 *    is above the 3000m AGL threshold for the entirety of these phases).
 * 8. Arrival rollout, taxi, and gate park: Computes linear deceleration rollout (300 to 50 km/h) and
 *    taxiing to the gate at 50 km/h.
 */
export function planFlight(departure: Airport, destination: Airport): FlightPlan {
  const distanceKm = getDistanceKm(departure, destination); // Geodesic distance in kilometers
  const route = DEPARTURE_ROUTES[departure.code] ?? null; // Departure procedure waypoints
  const arrival = getArrivalRoute(destination.code); // Arrival procedure waypoints

  const elev = departure.elevation; // MSL elevation of departure airport
  const destElev = destination.elevation; // MSL elevation of destination airport

  // ── Gate & runway holds (Fixed durations) ──────────────────────────────────
  const gateTime = route.gateHoldTime;
  const runwayHoldTime = route.runwayHoldTime;

  // ── Departure Taxi (Constant ground speed: 50 km/h) ──────────────────────
  // Route: Gate Stand -> Taxiway Intersections -> Runway Threshold
  const taxiPath = [route.gate, ...route.taxiWaypoints, route.runwayThreshold];
  const taxiDistM = computePathDistance(taxiPath); // Summed segment lengths
  const taxiTime = taxiDistM / MAX_GROUND_SPEED_MS; // t = s / v

  // ── Takeoff Roll (Constant acceleration from 0 → 300 km/h) ───────────
  // Route: Runway Threshold -> Runway Centerline Waypoints -> Rotation Point
  const rwyPath = [route.runwayThreshold, ...route.runwayWaypoints, route.liftoffPoint];
  const rwyDistM = computePathDistance(rwyPath); // Runway roll length
  // Physics of uniform acceleration from rest (v_initial = 0):
  // v_average = (v_initial + v_final) / 2 = v_final / 2.
  // Hence: Time = distance / v_average = 2 · distance / v_final (v_final = LIFTOFF_SPEED_MS)
  const takeoffTime = 2 * rwyDistM / LIFTOFF_SPEED_MS;

  // ── Climb Geometry (Straight climb along runway heading) ─────────────────
  // Calculate the runway heading (bearing) based on the last segment of the runway
  const prevPoint = route.runwayWaypoints[route.runwayWaypoints.length - 1] ?? route.runwayThreshold;
  const runwayHeadingRad = computeBearingRad(prevPoint, route.liftoffPoint);

  // Project the climb exit waypoint straight ahead along the runway vector
  const climbExitLon = route.liftoffPoint.lon + Math.sin(runwayHeadingRad) * CLIMB_DISTANCE_DEG;
  const climbExitLat = route.liftoffPoint.lat + Math.cos(runwayHeadingRad) * CLIMB_DISTANCE_DEG;

  // Horizontal climb distance over the earth's surface
  const climbHorizDistM = haversineDistMeters(
    route.liftoffPoint.lat, route.liftoffPoint.lon,
    climbExitLat, climbExitLon
  );

  const climbStartAlt = elev + 50; // Climb begins 50m above airport elevation (liftoff rotation complete)
  const climbEndAlt = elev + SPEED_RAMP_END_AGL; // Climb phase ends at 3000m AGL (above departure airport)

  // Compute duration using numerical integration to account for the exponential speed ramp-up
  const climbTime = computeVariableSpeedDuration(
    climbHorizDistM, climbStartAlt, climbEndAlt, elev
  );

  // ── Approach & Landing Geometry (Destination runway alignment) ───────────
  const landingTargetLat = arrival!.touchdownPoint.lat;
  const landingTargetLon = arrival!.touchdownPoint.lon;

  // Compute the landing runway direction. If an arrival route is available,
  // we compute the reverse bearing of the runway. Otherwise, we mirror the airport runway heading (180 deg offset).
  const destHeadingRad = arrival
    ? computeBearingRad(
        arrival.rolloutWaypoints[0] ?? arrival.runwayEnd,
        arrival.touchdownPoint
      )
    : Cesium.Math.toRadians((destination.runwayHeading + 180) % 360);

  // Project the approach entry point backward along the landing heading vector.
  // This aligns the aircraft perfectly straight on final approach for an automatic instrument landing (ILS).
  const approachEntryLon = landingTargetLon + Math.sin(destHeadingRad) * APPROACH_DISTANCE_DEG;
  const approachEntryLat = landingTargetLat + Math.cos(destHeadingRad) * APPROACH_DISTANCE_DEG;

  const approachHorizDistM = haversineDistMeters(
    approachEntryLat, approachEntryLon,
    landingTargetLat, landingTargetLon
  );

  const approachStartAlt = destElev + SPEED_RAMP_END_AGL; // Approach begins at 3000m AGL (above destination airport)
  const approachEndAlt = destElev; // Approach ends at touchdown point elevation

  // Compute landing approach time using quadratic easing (easeOutQuad) to model flight leveling (flare) before touchdown
  const landingTime = computeVariableSpeedDuration(
    approachHorizDistM, approachStartAlt, approachEndAlt, destElev, 100, easeOutQuad
  );

  // ── Great-Circle Cruise & Descent ────────────────────────────────────
  // Connect the climb exit and approach entry waypoints with a geodesic line (Great Circle path)
  const gcDistM = haversineDistMeters(
    climbExitLat, climbExitLon,
    approachEntryLat, approachEntryLon
  );
  
  // Partition the geodesic path:
  // - 80% is allocated to high-altitude Cruise (climbing further to CRUISE_ALTITUDE = 10,000m)
  // - 20% is allocated to Descent (gradual drop from 10,000m down to approach entry altitude of 3000m AGL)
  const cruiseDistM = gcDistM * 0.8;
  const descentDistM = gcDistM * 0.2;

  // Cruise duration (distance / max speed). We enforce a minimum duration (MIN_CRUISE_DURATION)
  // to ensure close-range airport pairs still display a visible cruising phase.
  const cruiseTime = Math.max(MIN_CRUISE_DURATION, cruiseDistM / MAX_SPEED_MS);

  // Descent duration (distance / max speed). Since the plane remains above 3000m AGL during descent,
  // it flies at maximum speed (900 km/h).
  const descentTime = descentDistM / MAX_SPEED_MS;

  // ── Holding/Alignment Patterns Check (Real-world dynamic alignment turns) ──
  const TURN_RATE_RAD_PER_SEC = Cesium.Math.toRadians(2); // Standard turn rate: 2 degrees per second
  const ALIGNMENT_THRESHOLD_RAD = Cesium.Math.toRadians(5); // Skip turn if within 5 degrees

  // 1. Climb Alignment: check if runway heading and start of cruise heading differ
  const cruiseStartHeading = computeBearingRad(
    { lon: climbExitLon, lat: climbExitLat },
    { lon: approachEntryLon, lat: approachEntryLat }
  );
  let climbHeadingDiff = Math.abs(runwayHeadingRad - cruiseStartHeading);
  if (climbHeadingDiff > Math.PI) {
    climbHeadingDiff = 2 * Math.PI - climbHeadingDiff;
  }
  const climbAlignTime = climbHeadingDiff > ALIGNMENT_THRESHOLD_RAD 
    ? climbHeadingDiff / TURN_RATE_RAD_PER_SEC 
    : 0;

  // 2. Descent Alignment: check if descent heading and landing runway heading differ
  const climbExitCartographic = Cesium.Cartographic.fromDegrees(climbExitLon, climbExitLat);
  const approachEntryCartographic = Cesium.Cartographic.fromDegrees(approachEntryLon, approachEntryLat);
  const geodesic = new Cesium.EllipsoidGeodesic(climbExitCartographic, approachEntryCartographic);
  const interp80 = geodesic.interpolateUsingFraction(0.80);
  const interp100 = geodesic.interpolateUsingFraction(1.00);
  const p80 = { lon: Cesium.Math.toDegrees(interp80.longitude), lat: Cesium.Math.toDegrees(interp80.latitude) };
  const p100 = { lon: Cesium.Math.toDegrees(interp100.longitude), lat: Cesium.Math.toDegrees(interp100.latitude) };
  const descentHeading = computeBearingRad(p80, p100);

  let descentHeadingDiff = Math.abs(descentHeading - destHeadingRad);
  if (descentHeadingDiff > Math.PI) {
    descentHeadingDiff = 2 * Math.PI - descentHeadingDiff;
  }
  const descentAlignTime = descentHeadingDiff > ALIGNMENT_THRESHOLD_RAD 
    ? descentHeadingDiff / TURN_RATE_RAD_PER_SEC 
    : 0;

  // ── Arrival Ground Phases (Rollout, Taxi, and Parking) ───────────────────
  let rolloutTime = 0, arrivalTaxiTime = 0, arrivalGateTime = 0;

  if (arrival) {
    // Rollout: Decelerating on the runway from touchdown speed (~300 km/h) to taxi speed (~50 km/h).
    // Computes duration analytically based on a linear speed deceleration profile.
    const rolloutPath = [arrival.touchdownPoint, ...arrival.rolloutWaypoints, arrival.runwayEnd];
    const rolloutDistM = computePathDistance(rolloutPath);
    rolloutTime = linearSpeedTotalTime(rolloutDistM, LIFTOFF_SPEED_MS, MAX_GROUND_SPEED_MS);

    // Arrival Taxi: Safe taxiing from the runway exit to the destination gate at 50 km/h.
    const arrTaxiPath = [arrival.runwayEnd, ...arrival.taxiToGateWaypoints, arrival.gate];
    const arrTaxiDistM = computePathDistance(arrTaxiPath);
    arrivalTaxiTime = arrTaxiDistM / MAX_GROUND_SPEED_MS;

    // Gate Parking: A short hold duration (5 seconds) to finalize the simulation flow.
    arrivalGateTime = ARRIVAL_GATE_HOLD;
  }

  // Compile individual phase durations
  const timing: FlightTiming = {
    gate: gateTime,
    taxi: taxiTime,
    runwayHold: runwayHoldTime,
    takeoff: takeoffTime,
    climb: climbTime,
    climbAlign: climbAlignTime,
    cruise: cruiseTime,
    descentAlign: descentAlignTime,
    descent: descentTime,
    landing: landingTime,
    rollout: rolloutTime,
    arrivalTaxi: arrivalTaxiTime,
    arrivalGate: arrivalGateTime,
  };

  // Sum all durations to determine the total flight duration
  const totalDuration = timing.gate + timing.taxi + timing.runwayHold +
    timing.takeoff + timing.climb + timing.climbAlign + timing.cruise +
    timing.descentAlign + timing.descent + timing.landing +
    timing.rollout + timing.arrivalTaxi + timing.arrivalGate;

  return { departure, destination, distanceKm, departureRoute: route, arrivalRoute: arrival, timing, totalDuration };
}

// ─── Create Flight ──────────────────────────────────────────

/**
 * Instantiates the complete 3D airplane entity in Cesium, generating a highly dense set of
 * time-stamped spatial coordinates (SampledPositionProperty) representing the entire flight path.
 * This function links UTC time values (JulianDate) with precise 3D cartographic coordinates (lon, lat, alt).
 *
 * Core spatial interpolation logic:
 * 1. Synchronizes the Cesium virtual timeline clock with the flight planning bounds (start to stop).
 * 2. Configures a SampledPositionProperty utilizing LinearApproximation. This provides efficient
 *    real-time rendering and integrates perfectly with Cesium's VelocityOrientationProperty to automatically
 *    compute the pitch, roll, and yaw angles of the 3D model on each frame.
 * 3. Establishes cumulative time offsets for each phase boundary (t_gateEnd, t_taxiEnd, ..., totalDuration).
 * 4. Ground departure: Generates gate and taxiing waypoints.
 * 5. Climb (100 samples): Projects a straight climb along the departure runway heading. Fits variable speed
 *    durations over the AGL speed curve.
 * 6. Cruise (1000 samples): Interpolates coordinate positions across the WGS84 geodesic elipsoid curve using
 *    Cesium.EllipsoidGeodesic, covering 0% to 80% of the route. Continues climbing to CRUISE_ALTITUDE (10,000m)
 *    during the first 10% of this phase.
 * 7. Descent (100 samples): Interpolates the final 20% of the geodesic path. Lowers altitude from 10,000m
 *    down to 3000m AGL using a smooth cubic ease-in-out curve (easeInOutCubic) to avoid abrupt pitch changes.
 * 8. Approach (100 samples): Intersects the approach entry point and touchdown waypoints. Applies easeOutQuad
 *    altitude easing to model flight leveling (flare) before touchdown.
 * 9. Ground arrival: Generates rollout, taxiway-to-gate, and gate-hold waypoints.
 */
export function createFlight(
  viewer: Cesium.Viewer,
  plan: FlightPlan
): FlightResult {
  const { departure, destination, timing, totalDuration, departureRoute, arrivalRoute } = plan;

  // Initialize starting JulianDate based on viewer clock time
  const start = viewer.clock.currentTime.clone();
  // Target completion time is exactly start time plus the total calculated duration
  const stop = Cesium.JulianDate.addSeconds(start, totalDuration, new Cesium.JulianDate());

  // Configure timeline range and playback parameters
  viewer.clock.startTime = start.clone();
  viewer.clock.stopTime = stop.clone();
  viewer.clock.currentTime = start.clone();
  viewer.clock.clockRange = Cesium.ClockRange.CLAMPED; // Hold the timeline at the end instead of looping

  // Initialize time-sampled 3D positioning property
  const position = new Cesium.SampledPositionProperty();
  position.setInterpolationOptions({
    interpolationDegree: 1,
    interpolationAlgorithm: Cesium.LinearApproximation, // Linear coordinate transitions between adjacent samples
  });

  const elev = departure.elevation;
  const destElev = destination.elevation;

  // Calculate cumulative phase time boundaries (seconds elapsed since simulation start)
  const t_gateEnd = timing.gate;                                      // Departure gate hold
  const t_taxiEnd = t_gateEnd + timing.taxi;                          // Safe ground taxiing
  const t_holdEnd = t_taxiEnd + timing.runwayHold;                    // Pre-takeoff check hold
  const t_takeoffEnd = t_holdEnd + timing.takeoff;                    // Runway takeoff roll
  const t_climbEnd = t_takeoffEnd + timing.climb;                      // Initial straight climb (3000m AGL)
  const t_climbAlignEnd = t_climbEnd + timing.climbAlign;              // Departure alignment turn
  const t_cruiseEnd = t_climbAlignEnd + timing.cruise;                  // Geodesic cruising (10000m MSL)
  const t_descentAlignEnd = t_cruiseEnd + timing.descentAlign;          // Arrival alignment turn
  const t_descentEnd = t_descentAlignEnd + timing.descent;              // Geodesic descent (3000m AGL)
  const t_landingEnd = t_descentEnd + timing.landing;                  // Touchdown point reached
  const t_rolloutEnd = t_landingEnd + timing.rollout;                  // Braking and deceleration rollout
  const t_arrivalTaxiEnd = t_rolloutEnd + timing.arrivalTaxi;          // Ground taxi to arrival gate
  // t_arrivalGateEnd = totalDuration                                  // Final parked hold at gate stand

  // ── Determine Departure Orientation & Liftoff Coordinates ────────
  let liftoffLat: number;
  let liftoffLon: number;
  let runwayHeadingRad: number;

  if (departureRoute) {
    liftoffLat = departureRoute.liftoffPoint.lat;
    liftoffLon = departureRoute.liftoffPoint.lon;
    // Derive runway heading from the final runway centerline segment
    const prevPoint = departureRoute.runwayWaypoints[departureRoute.runwayWaypoints.length - 1] ?? departureRoute.runwayThreshold;
    runwayHeadingRad = computeBearingRad(prevPoint, departureRoute.liftoffPoint);
  } else {
    // Fallback simple heading projection if no route configuration exists
    runwayHeadingRad = Cesium.Math.toRadians(departure.runwayHeading);
    liftoffLon = departure.lon + Math.sin(runwayHeadingRad) * 0.02;
    liftoffLat = departure.lat + Math.cos(runwayHeadingRad) * 0.02;
  }

  // Calculate straight-line climb exit waypoint aligned with runway heading
  const climbExitLon = liftoffLon + Math.sin(runwayHeadingRad) * CLIMB_DISTANCE_DEG;
  const climbExitLat = liftoffLat + Math.cos(runwayHeadingRad) * CLIMB_DISTANCE_DEG;

  // Set altitude thresholds (AGL-based heights translated to MSL)
  const climbStartAlt = elev + 50;
  const climbEndAlt = elev + SPEED_RAMP_END_AGL;          // 3000m AGL
  const approachEntryAlt = destElev + SPEED_RAMP_END_AGL;  // 3000m AGL

  // Configure arrival touchdown targets
  let landingTargetLat: number;
  let landingTargetLon: number;
  let landingTargetElev: number;

  landingTargetLat = arrivalRoute!.touchdownPoint.lat;
  landingTargetLon = arrivalRoute!.touchdownPoint.lon;
  landingTargetElev = destElev;

  // Determine approach course heading (aligned with landing runway axis)
  const destHeadingRad = arrivalRoute
    ? computeBearingRad(arrivalRoute.rolloutWaypoints[0] ?? arrivalRoute.runwayEnd, arrivalRoute.touchdownPoint)
    : Cesium.Math.toRadians((destination.runwayHeading + 180) % 360);

  // Project the approach entry point backward along the landing heading vector
  const approachEntryLon = landingTargetLon + Math.sin(destHeadingRad) * APPROACH_DISTANCE_DEG;
  const approachEntryLat = landingTargetLat + Math.cos(destHeadingRad) * APPROACH_DISTANCE_DEG;

  const climbExitCartographic = Cesium.Cartographic.fromDegrees(climbExitLon, climbExitLat);
  const approachEntryCartographic = Cesium.Cartographic.fromDegrees(approachEntryLon, approachEntryLat);
  const geodesicTemp = new Cesium.EllipsoidGeodesic(climbExitCartographic, approachEntryCartographic);

  // 1. Climb Alignment calculation
  const cruiseStartHeading = computeBearingRad(
    { lon: climbExitLon, lat: climbExitLat },
    { lon: approachEntryLon, lat: approachEntryLat }
  );
  let climbAlignEndLon = climbExitLon;
  let climbAlignEndLat = climbExitLat;
  let climbHeadingDiffVal = cruiseStartHeading - runwayHeadingRad;
  while (climbHeadingDiffVal > Math.PI) climbHeadingDiffVal -= 2 * Math.PI;
  while (climbHeadingDiffVal < -Math.PI) climbHeadingDiffVal += 2 * Math.PI;

  const TURN_RATE = Cesium.Math.toRadians(2); // 2 degrees per second
  const vClimbAlign = getSpeedForAltitude(climbEndAlt, elev);
  const rClimbAlignM = vClimbAlign / TURN_RATE;
  const rClimbAlignDeg = rClimbAlignM / 111320;
  
  let climbCenterLon = 0, climbCenterLat = 0, climbStartAngle = 0;
  const isClimbRightTurn = climbHeadingDiffVal > 0;
  const climbDiffAbs = Math.abs(climbHeadingDiffVal);

  if (timing.climbAlign > 0) {
    if (isClimbRightTurn) {
      climbCenterLon = climbExitLon + rClimbAlignDeg * Math.cos(runwayHeadingRad);
      climbCenterLat = climbExitLat - rClimbAlignDeg * Math.sin(runwayHeadingRad);
      climbStartAngle = Math.atan2(Math.sin(runwayHeadingRad), -Math.cos(runwayHeadingRad));
    } else {
      climbCenterLon = climbExitLon - rClimbAlignDeg * Math.cos(runwayHeadingRad);
      climbCenterLat = climbExitLat + rClimbAlignDeg * Math.sin(runwayHeadingRad);
      climbStartAngle = Math.atan2(-Math.sin(runwayHeadingRad), Math.cos(runwayHeadingRad));
    }
    const endAngle = isClimbRightTurn ? climbStartAngle - climbDiffAbs : climbStartAngle + climbDiffAbs;
    climbAlignEndLon = climbCenterLon + rClimbAlignDeg * Math.cos(endAngle);
    climbAlignEndLat = climbCenterLat + rClimbAlignDeg * Math.sin(endAngle);
  }

  // 2. Descent Alignment calculation (backwards projection from approachEntry)
  const interpPrev = geodesicTemp.interpolateUsingFraction(0.99);
  const interpEnd = geodesicTemp.interpolateUsingFraction(1.0);
  const arrivalGeodesicHeading = computeBearingRad(
    { lon: Cesium.Math.toDegrees(interpPrev.longitude), lat: Cesium.Math.toDegrees(interpPrev.latitude) },
    { lon: Cesium.Math.toDegrees(interpEnd.longitude), lat: Cesium.Math.toDegrees(interpEnd.latitude) }
  );

  let descentAlignStartLon = approachEntryLon;
  let descentAlignStartLat = approachEntryLat;
  let descentHeadingDiffVal = destHeadingRad - arrivalGeodesicHeading;
  while (descentHeadingDiffVal > Math.PI) descentHeadingDiffVal -= 2 * Math.PI;
  while (descentHeadingDiffVal < -Math.PI) descentHeadingDiffVal += 2 * Math.PI;

  const vDescentAlign = getSpeedForAltitude(approachEntryAlt, destElev);
  const rDescentAlignM = vDescentAlign / TURN_RATE;
  const rDescentAlignDeg = rDescentAlignM / 111320;

  let descentCenterLon = 0, descentCenterLat = 0, descentStartAngle = 0;
  const isDescentRightTurn = descentHeadingDiffVal > 0;
  const descentDiffAbs = Math.abs(descentHeadingDiffVal);

  if (timing.descentAlign > 0) {
    if (isDescentRightTurn) {
      descentCenterLon = approachEntryLon + rDescentAlignDeg * Math.cos(destHeadingRad);
      descentCenterLat = approachEntryLat - rDescentAlignDeg * Math.sin(destHeadingRad);
      const endAngle = Math.atan2(Math.sin(destHeadingRad), -Math.cos(destHeadingRad));
      descentStartAngle = endAngle + descentDiffAbs;
    } else {
      descentCenterLon = approachEntryLon - rDescentAlignDeg * Math.cos(destHeadingRad);
      descentCenterLat = approachEntryLat + rDescentAlignDeg * Math.sin(destHeadingRad);
      const endAngle = Math.atan2(-Math.sin(destHeadingRad), Math.cos(destHeadingRad));
      descentStartAngle = endAngle - descentDiffAbs;
    }
    descentAlignStartLon = descentCenterLon + rDescentAlignDeg * Math.cos(descentStartAngle);
    descentAlignStartLat = descentCenterLat + rDescentAlignDeg * Math.sin(descentStartAngle);
  }

  // Initialize the final geodesic path connecting climbAlignEnd with descentAlignStart
  const finalClimbAlignEndCart = Cesium.Cartographic.fromDegrees(climbAlignEndLon, climbAlignEndLat);
  const finalDescentAlignStartCart = Cesium.Cartographic.fromDegrees(descentAlignStartLon, descentAlignStartLat);
  const geodesic = new Cesium.EllipsoidGeodesic(finalClimbAlignEndCart, finalDescentAlignStartCart);

  // Helper utility to write a 3D position sample at a specific elapsed time offset
  function addSample(elapsed: number, lon: number, lat: number, alt: number): void {
    const time = Cesium.JulianDate.addSeconds(start, elapsed, new Cesium.JulianDate());
    position.addSample(time, Cesium.Cartesian3.fromDegrees(lon, lat, alt));
  }

  // ── DEPARTURE PROCEDURE (Gate -> Taxi -> Runway Hold -> Takeoff Roll) ───────────
  buildRouteDeparture(departureRoute!, elev, addSample, timing);

  // ── STRAIGHT CLIMB (Liftoff point → 3000m AGL along runway axis) ────────────────
  const climbSamples = 100; // High sampling rate for smooth vertical pitch representation
  const climbHorizDistM = haversineDistMeters(liftoffLat, liftoffLon, climbExitLat, climbExitLon);
  // Get time timestamps per sample utilizing numerical speed profile integration
  const climbCumTimes = computeCumulativeTimes(
    climbHorizDistM, climbStartAlt, climbEndAlt, elev, climbSamples
  );

  for (let i = 0; i <= climbSamples; i++) {
    const frac = i / climbSamples;
    const elapsed = t_takeoffEnd + climbCumTimes[i];
    // Interpolate coordinates linearly on the straight climb vector
    const lon = lerp(liftoffLon, climbExitLon, frac);
    const lat = lerp(liftoffLat, climbExitLat, frac);
    const alt = lerp(climbStartAlt, climbEndAlt, frac);
    addSample(elapsed, lon, lat, alt);
  }

  // ── DEPARTURE ALIGNMENT TURN (climb_align) ──────────────────────────────────
  if (timing.climbAlign > 0) {
    const holdSamples = 60;
    let climbHeadingDiffVal = cruiseStartHeading - runwayHeadingRad;
    while (climbHeadingDiffVal > Math.PI) climbHeadingDiffVal -= 2 * Math.PI;
    while (climbHeadingDiffVal < -Math.PI) climbHeadingDiffVal += 2 * Math.PI;

    const isRightTurn = climbHeadingDiffVal > 0;
    const diffAbs = Math.abs(climbHeadingDiffVal);

    for (let i = 0; i <= holdSamples; i++) {
      const frac = i / holdSamples;
      const elapsed = t_climbEnd + frac * timing.climbAlign;
      const angle = isRightTurn ? climbStartAngle - frac * diffAbs : climbStartAngle + frac * diffAbs;
      const lon = climbCenterLon + rClimbAlignDeg * Math.cos(angle);
      const lat = climbCenterLat + rClimbAlignDeg * Math.sin(angle);
      const alt = climbEndAlt;
      addSample(elapsed, lon, lat, alt);
    }
  }

  // ── PLANETARY CRUISE (Great-circle arc across Earth's ellipsoid) ─────────────────
  const cruiseSamples = 1000; // Dense sampling over long range to prevent polygonal path artifacts
  for (let i = 0; i <= cruiseSamples; i++) {
    const frac = i / cruiseSamples;
    const elapsed = t_climbAlignEnd + frac * timing.cruise;

    // Cruising spans the first 80% of the geodesic great-circle arc
    const gcFrac = lerp(0.0, 0.80, frac);
    const interp = geodesic.interpolateUsingFraction(gcFrac); // Spherical ellipsoid interpolation
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);

    let alt = CRUISE_ALTITUDE;
    // Perform a smooth, gradual climb from climbEndAlt (3000m AGL) up to CRUISE_ALTITUDE (10,000m)
    // during the first 10% of the cruising timeline
    if (frac < 0.1) {
      alt = lerp(climbEndAlt, CRUISE_ALTITUDE, frac / 0.1);
    }

    addSample(elapsed, lon, lat, alt);
  }

  // ── HIGH-ALTITUDE DESCENT (Geodesic descent to approach corridor) ───────────────
  const descentSamples = 100;
  for (let i = 0; i <= descentSamples; i++) {
    const frac = i / descentSamples;
    const elapsed = t_cruiseEnd + frac * timing.descent;

    // Descent spans the remaining 20% (from 80% to 100%) of the geodesic great-circle arc
    const gcFrac = lerp(0.80, 1.0, frac);
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);
    // Smoothly transition altitude from 10,000m down to approachEntryAlt (3000m AGL)
    // using a cubic easing function (easeInOutCubic) to avoid abrupt pitch-down visual anomalies
    const alt = lerp(CRUISE_ALTITUDE, approachEntryAlt, easeInOutCubic(frac));

    addSample(elapsed, lon, lat, alt);
  }

  // ── ARRIVAL ALIGNMENT TURN (descent_align) ──────────────────────────────────
  if (timing.descentAlign > 0) {
    const holdSamples = 60;
    for (let i = 0; i <= holdSamples; i++) {
      const frac = i / holdSamples;
      const elapsed = t_descentEnd + frac * timing.descentAlign;
      const angle = isDescentRightTurn ? descentStartAngle - frac * descentDiffAbs : descentStartAngle + frac * descentDiffAbs;
      const lon = descentCenterLon + rDescentAlignDeg * Math.cos(angle);
      const lat = descentCenterLat + rDescentAlignDeg * Math.sin(angle);
      const alt = approachEntryAlt; // hold terminal approach entry altitude (3000m AGL)
      addSample(elapsed, lon, lat, alt);
    }
  }

  // ── FINAL APPROACH & LANDING (Glide slope down to touchdown point) ───────────────
  const landingSamples = 100;
  const approachHorizDistM = haversineDistMeters(
    approachEntryLat, approachEntryLon,
    landingTargetLat, landingTargetLon
  );
  // Get time timestamps per sample utilizing landing speed numerical integration
  const landingCumTimes = computeCumulativeTimes(
    approachHorizDistM, approachEntryAlt, landingTargetElev, destElev, landingSamples, easeOutQuad
  );

  for (let i = 0; i <= landingSamples; i++) {
    const frac = i / landingSamples;
    const elapsed = t_descentAlignEnd + landingCumTimes[i];
    // Interpolate coordinates along the straight-line glide path centerline
    const lon = lerp(approachEntryLon, landingTargetLon, frac);
    const lat = lerp(approachEntryLat, landingTargetLat, frac);
    // Eased altitude curve (easeOutQuad) flattens out heights right before touchdown,
    // realistically mimicking a landing flare maneuver
    const alt = lerp(approachEntryAlt, landingTargetElev, easeOutQuad(frac));
    addSample(elapsed, lon, lat, alt);
  }

  // ── ARRIVAL GROUND PROCEDURE (Runway Rollout -> Taxiway -> Gate Stand) ───────────
  if (arrivalRoute) {
    buildRouteArrival(arrivalRoute, destElev, addSample, timing, t_landingEnd);
  }

  const entity = viewer.entities.add({
    name: `Flight ${departure.code} → ${destination.code}`,
    availability: new Cesium.TimeIntervalCollection([
      new Cesium.TimeInterval({ start, stop }),
    ]),
    position: position,
    orientation: new Cesium.VelocityOrientationProperty(position),
    model: {
      uri: "/plane.glb",
      minimumPixelSize: 80,
      maximumScale: 500,
      silhouetteColor: Cesium.Color.fromCssColorString("#3b82f6"),
      silhouetteSize: 1,
    },
    path: {
      resolution: 1,
      material: new Cesium.PolylineGlowMaterialProperty({
        glowPower: 0.15,
        color: Cesium.Color.fromCssColorString("#60a5fa"),
      }),
      width: 8,
      leadTime: 0,
      trailTime: totalDuration,
    },
  });

  function getPhase(currentTime: Cesium.JulianDate): FlightPhase {
    const elapsed = Cesium.JulianDate.secondsDifference(currentTime, start);
    if (elapsed < 0) return "preflight";
    if (timing.gate > 0 && elapsed <= t_gateEnd) return "gate";
    if (timing.taxi > 0 && elapsed <= t_taxiEnd) return "taxi";
    if (timing.runwayHold > 0 && elapsed <= t_holdEnd) return "runway_hold";
    if (elapsed <= t_takeoffEnd) return "takeoff";
    if (elapsed <= t_climbEnd) return "climb";
    if (timing.climbAlign > 0 && elapsed <= t_climbAlignEnd) return "climb_align";
    if (elapsed <= t_cruiseEnd) return "cruise";
    if (elapsed <= t_descentEnd) return "descent";
    if (timing.descentAlign > 0 && elapsed <= t_descentAlignEnd) return "descent_align";
    if (elapsed <= t_landingEnd) return "landing";
    if (timing.rollout > 0 && elapsed <= t_rolloutEnd) return "rollout";
    if (timing.arrivalTaxi > 0 && elapsed <= t_arrivalTaxiEnd) return "arrival_taxi";
    if (timing.arrivalGate > 0 && elapsed < totalDuration) return "arrival_gate";
    return "arrived";
  }

  // Departure camera info
  let departureCamera: FlightResult["departureCamera"] = null;
  if (departureRoute) {
    const firstTaxiTarget = departureRoute.taxiWaypoints[0] ?? departureRoute.runwayThreshold;
    const heading = computeBearingRad(departureRoute.gate, firstTaxiTarget);
    departureCamera = {
      gatePosition: departureRoute.gate,
      initialHeading: heading,
    };
  }

  return { entity, plan, startTime: start, stopTime: stop, getPhase, departureCamera };
}

// ─── Build Route Departure ──────────────────────────────────

type AddSampleFn = (elapsed: number, lon: number, lat: number, alt: number) => void;

/**
 * Build samples for a route-based departure: gate → taxi → hold → takeoff roll → liftoff.
 * Taxi at 50 km/h, takeoff roll with acceleration from 0 to 300 km/h.
 */
/**
 * Generates sample coordinates for the ground departure route: gate hold → taxiing → runway hold → takeoff roll → liftoff rotation.
 *
 * Ground motion kinematics & mathematics:
 * 1. Gate Hold Stand: The aircraft is stationary. To allow Cesium's VelocityOrientationProperty to resolve a non-zero
 *    velocity vector, we inject a microscopic coordinate drift (DRIFT) aligned with the taxi heading. This prevents
 *    the 3D aircraft model orientation from resetting or spinning during the gate wait period.
 * 2. Taxi: Partitions the taxi route into discrete GPS path segments. Map timing linearly to segment distance fractions
 *    since ground taxiing occurs at a constant ground speed of 50 km/h. Sub-samples each segment to ensure smooth ground turns.
 * 3. Runway Hold: Similar to gate hold, injects micro-drift along the runway heading to preserve 3D model orientation.
 * 4. Takeoff Roll (Acceleration): Models uniform linear acceleration from 0 to 300 km/h (LIFTOFF_SPEED).
 *    From physics: distance is proportional to time squared (s ~ t^2), meaning time elapsed is proportional to the square
 *    root of distance travelled (t ~ sqrt(s)). For each sub-sampled point, we compute the distance fraction (distFrac)
 *    and calculate time as: elapsed = t_start + t_takeoff · sqrt(distFrac). This yields a realistic, physically accurate
 *    takeoff roll visualization.
 * 5. Liftoff Rotation: In the final 50% of the last runway segment, the plane begins to rotate. We smoothly transition
 *    the altitude from runway elevation up to elevation + 50m using a quadratic ease-in function (easeInQuad).
 */
function buildRouteDeparture(
  route: DepartureRoute,
  elevation: number,
  addSample: AddSampleFn,
  timing: FlightTiming
): void {
  let t = 0;

  // ── GATE HOLD (Stationary with micro-drift for 3D heading stability) ───────
  const firstTarget = route.taxiWaypoints[0] ?? route.runwayThreshold;
  const gateBearing = computeBearingRad(route.gate, firstTarget); // Initial heading azimuth
  const DRIFT = 0.0000005; // 0.05 micro-degrees per second drift to maintain orientation vector

  for (let i = 0; i <= 5; i++) {
    const frac = i / 5;
    const elapsed = frac * timing.gate;
    addSample(
      elapsed,
      route.gate.lon + Math.sin(gateBearing) * DRIFT * elapsed,
      route.gate.lat + Math.cos(gateBearing) * DRIFT * elapsed,
      elevation
    );
  }
  t = timing.gate;

  // ── GROUND TAXI (Constant ground speed: 50 km/h) ────────────────────────
  const taxiPath: RouteWaypoint[] = [
    route.gate,
    ...route.taxiWaypoints,
    route.runwayThreshold,
  ];

  // Calculate horizontal distances of individual taxiway legs
  const taxiSegDists: number[] = [];
  for (let i = 0; i < taxiPath.length - 1; i++) {
    taxiSegDists.push(waypointDistMeters(taxiPath[i], taxiPath[i + 1]));
  }
  const totalTaxiDist = taxiSegDists.reduce((a, b) => a + b, 0);

  let taxiCumDist = 0;
  for (let seg = 0; seg < taxiPath.length - 1; seg++) {
    const from = taxiPath[seg];
    const to = taxiPath[seg + 1];
    const segDist = taxiSegDists[seg];

    const subStart = seg === 0 ? 0 : 1;
    // Divide each taxi segment into 3 sub-sections to ensure smooth, round wheel turns
    for (let sub = subStart; sub <= 3; sub++) {
      const frac = sub / 3;
      const distAtSub = taxiCumDist + frac * segDist;
      const timeFrac = totalTaxiDist > 0 ? distAtSub / totalTaxiDist : 0;
      const elapsed = t + timeFrac * timing.taxi; // Time is linear to distance
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);
      addSample(elapsed, lon, lat, elevation);
    }
    taxiCumDist += segDist;
  }
  t += timing.taxi;

  // ── PRE-TAKEOFF RUNWAY HOLD (Short hold on runway centerline) ──────────────
  const firstRwyTarget = route.runwayWaypoints[0] ?? route.liftoffPoint;
  const rwyBearing = computeBearingRad(route.runwayThreshold, firstRwyTarget); // Runway course bearing

  for (let i = 0; i <= 3; i++) {
    const frac = i / 3;
    const elapsed = t + frac * timing.runwayHold;
    // Inject micro-drift along the runway centerline to prevent orientation snapping
    addSample(
      elapsed,
      route.runwayThreshold.lon + Math.sin(rwyBearing) * DRIFT * (frac * timing.runwayHold),
      route.runwayThreshold.lat + Math.cos(rwyBearing) * DRIFT * (frac * timing.runwayHold),
      elevation
    );
  }
  t += timing.runwayHold;

  // ── TAKEOFF ROLL (Uniform linear acceleration: 0 → 300 km/h) ───────────────
  const rwyPath: RouteWaypoint[] = [
    route.runwayThreshold,
    ...route.runwayWaypoints,
    route.liftoffPoint,
  ];

  const rwySegDists: number[] = [];
  for (let i = 0; i < rwyPath.length - 1; i++) {
    rwySegDists.push(waypointDistMeters(rwyPath[i], rwyPath[i + 1]));
  }
  const totalRwyDist = rwySegDists.reduce((a, b) => a + b, 0);

  let rwyCumDist = 0;
  for (let seg = 0; seg < rwyPath.length - 1; seg++) {
    const from = rwyPath[seg];
    const to = rwyPath[seg + 1];
    const segDist = rwySegDists[seg];
    const isLastSegment = seg === rwyPath.length - 2;

    const subStart = seg === 0 ? 0 : 1;
    for (let sub = subStart; sub <= 4; sub++) {
      const frac = sub / 4;
      const distAtSub = rwyCumDist + frac * segDist;
      const distFrac = totalRwyDist > 0 ? distAtSub / totalRwyDist : 0;
      const elapsed = t + timing.takeoff * Math.sqrt(distFrac);
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);

      let alt = elevation;
      if (isLastSegment && frac > 0.5) {
        const liftFrac = (frac - 0.5) / 0.5;
        alt = elevation + easeInQuad(liftFrac) * 50;
      }
      addSample(elapsed, lon, lat, alt);
    }
    rwyCumDist += segDist;
  }
}

// ─── Build Route Arrival ────────────────────────────────────

/**
 * Generates sample coordinates for the ground arrival route: runway rollout deceleration → taxi to gate stand → arrival gate park.
 *
 * Arrival ground kinematics & mathematics:
 * 1. Runway Rollout (Deceleration): Decelerates the aircraft from touchdown speed (~300 km/h) down to
 *    taxiing speed (~50 km/h). Uses an analytical time fraction calculation (`linearSpeedTimeFrac`) for linear velocity deceleration
 *    (constant braking force). Time increases logarithmically as speed drops, delivering exceptionally smooth, lifelike braking.
 * 2. Arrival Taxi: Safely guides the plane from the runway exit down taxiway segments to the destination gate at a constant
 *    speed of 50 km/h. Divides segments to ensure ground alignment is maintained at all points.
 * 3. Gate Park Hold: Concludes the flight. Similar to departure gate hold, a micro-drift (DRIFT) coordinate offset is injected
 *    to preserve the non-zero speed vector, maintaining a stable 3D model yaw orientation in Cesium after parking.
 */
function buildRouteArrival(
  route: ArrivalRoute,
  elevation: number,
  addSample: AddSampleFn,
  timing: FlightTiming,
  tStart: number
): void {
  let t = tStart;

  // ── RUNWAY ROLLOUT DECELERATION (Braking from 300 to 50 km/h) ───────────────
  const rolloutPath: RouteWaypoint[] = [
    route.touchdownPoint,
    ...route.rolloutWaypoints,
    route.runwayEnd,
  ];

  const rollSegDists: number[] = [];
  for (let i = 0; i < rolloutPath.length - 1; i++) {
    rollSegDists.push(waypointDistMeters(rolloutPath[i], rolloutPath[i + 1]));
  }
  const totalRollDist = rollSegDists.reduce((a, b) => a + b, 0);

  let rolloutCumDist = 0;
  for (let seg = 0; seg < rolloutPath.length - 1; seg++) {
    const from = rolloutPath[seg];
    const to = rolloutPath[seg + 1];
    const segDist = rollSegDists[seg];

    const subStart = seg === 0 ? 0 : 1;
    for (let sub = subStart; sub <= 4; sub++) {
      const frac = sub / 4;
      const distAtSub = rolloutCumDist + frac * segDist;
      const distFrac = totalRollDist > 0 ? distAtSub / totalRollDist : 0;
      const timeFrac = linearSpeedTimeFrac(distFrac, LIFTOFF_SPEED_MS, MAX_GROUND_SPEED_MS);
      const elapsed = t + timeFrac * timing.rollout;
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);
      addSample(elapsed, lon, lat, elevation);
    }
    rolloutCumDist += segDist;
  }
  t += timing.rollout;

  // ── ARRIVAL TAXI (Constant ground speed: 50 km/h) ────────────
  const taxiPath: RouteWaypoint[] = [
    route.runwayEnd,
    ...route.taxiToGateWaypoints,
    route.gate,
  ];

  const taxiSegDists: number[] = [];
  for (let i = 0; i < taxiPath.length - 1; i++) {
    taxiSegDists.push(waypointDistMeters(taxiPath[i], taxiPath[i + 1]));
  }
  const totalTaxiDist = taxiSegDists.reduce((a, b) => a + b, 0);

  let taxiCumDist = 0;
  for (let seg = 0; seg < taxiPath.length - 1; seg++) {
    const from = taxiPath[seg];
    const to = taxiPath[seg + 1];
    const segDist = taxiSegDists[seg];

    const subStart = seg === 0 ? 0 : 1;
    for (let sub = subStart; sub <= 3; sub++) {
      const frac = sub / 3;
      const distAtSub = taxiCumDist + frac * segDist;
      const timeFrac = totalTaxiDist > 0 ? distAtSub / totalTaxiDist : 0;
      const elapsed = t + timeFrac * timing.arrivalTaxi;
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);
      addSample(elapsed, lon, lat, elevation);
    }
    taxiCumDist += segDist;
  }
  t += timing.arrivalTaxi;

  // ── ARRIVAL GATE PARKED (Stationary with micro-drift for heading stability) ───────
  const lastTaxiPoint = route.taxiToGateWaypoints[route.taxiToGateWaypoints.length - 1] ?? route.runwayEnd;
  const gateBearing = computeBearingRad(lastTaxiPoint, route.gate);
  const DRIFT = 0.0000005;

  for (let i = 0; i <= 5; i++) {
    const frac = i / 5;
    const elapsed = t + frac * timing.arrivalGate;
    addSample(
      elapsed,
      route.gate.lon + Math.sin(gateBearing) * DRIFT * (frac * timing.arrivalGate),
      route.gate.lat + Math.cos(gateBearing) * DRIFT * (frac * timing.arrivalGate),
      elevation
    );
  }
}

// ─── Math Helpers ───────────────────────────────────────────

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

function easeInQuad(t: number): number {
  return t * t;
}

function easeOutQuad(t: number): number {
  return t * (2 - t);
}

function easeInOutCubic(t: number): number {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

function computeBearingRad(from: RouteWaypoint, to: RouteWaypoint): number {
  const dLon = (to.lon - from.lon) * Math.PI / 180;
  const lat1 = from.lat * Math.PI / 180;
  const lat2 = to.lat * Math.PI / 180;
  const y = Math.sin(dLon) * Math.cos(lat2);
  const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
  return (Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI);
}
