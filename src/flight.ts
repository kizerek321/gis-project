/**
 * Flight path engine — generates realistic departure → cruise → landing paths.
 *
 * This is basically the core module that handles everything about how
 * the plane moves from point A to point B. It calculates positions,
 * altitudes, speeds, and timings for every phase of the flight.
 *
 * Speed profile (AGL — above ground level):
 *   Ground:      max 50 km/h  (taxi, rollout end)
 *   0–400m:      300 km/h     (liftoff / final approach)
 *   400–800m:    300 → 450 km/h
 *   800–3000m:   450 → 900 km/h (main speed increase)
 *   3000m+:      900 km/h     (cruise, never exceeded)
 *
 * Climb phase goes all the way from ground to cruise altitude (10 000 m)
 * over a ~100 km horizontal distance (~5.7° angle, realistic for airliners).
 * Landing mirrors the climb speed profile in reverse.
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
  | "cruise"        // Great-circle at cruise altitude
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
  cruise: number;
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
const CLIMB_DISTANCE_M = 100_000;    // 100 km straight climb distance
const APPROACH_DISTANCE_DEG = 0.90;  // ~100 km approach distance (matches climb geometry)

// Fixed durations
const ARRIVAL_GATE_HOLD = 5;         // seconds parked at destination gate
const MIN_CRUISE_DURATION = 15;      // seconds minimum cruise time

// ─── Speed Profile ──────────────────────────────────────────

/**
 * Returns the airplane speed (m/s) based on how high it is above the ground.
 *
 * So basically the idea here is that planes don't just instantly go fast.
 * They speed up gradually as they climb higher. This function figures out
 * what speed the plane should be at for any given altitude.
 *
 * The speed ramps up in bands:
 *   On the ground:      50 km/h  (just taxiing around)
 *   0 – 400m AGL:      300 km/h (right after liftoff, still slow-ish)
 *   400 – 800m AGL:    300→450  (starting to pick up speed)
 *   800 – 3000m AGL:   450→900  (this is where most acceleration happens)
 *   Above 3000m AGL:   900 km/h (full cruise speed, stays constant)
 *
 * @param altitude  - current altitude in meters (MSL = above sea level)
 * @param groundElev - ground elevation at the airport (to calc AGL)
 * @returns speed in m/s
 */
function getSpeedForAltitude(altitude: number, groundElev: number): number {
  // AGL = Above Ground Level, so we subtract the airport elevation
  const agl = altitude - groundElev;
  if (agl <= 0) return MAX_GROUND_SPEED_MS;
  if (agl <= SPEED_RAMP_START_AGL) return LIFTOFF_SPEED_MS;
  if (agl <= SPEED_RAMP_MID_AGL) {
    // Linear interpolation between liftoff speed and mid-climb speed
    const frac = (agl - SPEED_RAMP_START_AGL) / (SPEED_RAMP_MID_AGL - SPEED_RAMP_START_AGL);
    return lerp(LIFTOFF_SPEED_MS, CLIMB_MID_SPEED_MS, frac);
  }
  if (agl <= SPEED_RAMP_END_AGL) {
    // Linear interpolation between mid-climb speed and cruise speed
    const frac = (agl - SPEED_RAMP_MID_AGL) / (SPEED_RAMP_END_AGL - SPEED_RAMP_MID_AGL);
    return lerp(CLIMB_MID_SPEED_MS, MAX_SPEED_MS, frac);
  }
  return MAX_SPEED_MS;
}

// ─── Distance Helpers ───────────────────────────────────────
// These functions calculate distances on Earth's surface.
// We use the Haversine formula which accounts for the Earth being round
// (not flat lol). It's super important for getting accurate distances
// between airports that might be thousands of km apart.

/**
 * Haversine distance between two waypoints in meters.
 * This is the "as the crow flies" distance along the Earth's surface.
 * We use it everywhere to figure out how far apart two points are.
 */
function waypointDistMeters(a: RouteWaypoint, b: RouteWaypoint): number {
  const R = 6371000; // Earth's radius in meters
  const dLat = (b.lat - a.lat) * Math.PI / 180;
  const dLon = (b.lon - a.lon) * Math.PI / 180;
  const lat1 = a.lat * Math.PI / 180;
  const lat2 = b.lat * Math.PI / 180;
  const sinDLat2 = Math.sin(dLat / 2);
  const sinDLon2 = Math.sin(dLon / 2);
  const h = sinDLat2 * sinDLat2 + Math.cos(lat1) * Math.cos(lat2) * sinDLon2 * sinDLon2;
  return R * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}

/**
 * Total path length through an array of waypoints (meters).
 * Basically just adds up the distance between each consecutive pair.
 * Used for taxi routes, runway paths, etc.
 */
function computePathDistance(waypoints: RouteWaypoint[]): number {
  let total = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    total += waypointDistMeters(waypoints[i], waypoints[i + 1]);
  }
  return total;
}

/** Convenience wrapper — same haversine but takes raw lat/lon numbers. */
function haversineDistMeters(lat1: number, lon1: number, lat2: number, lon2: number): number {
  return waypointDistMeters({ lat: lat1, lon: lon1 }, { lat: lat2, lon: lon2 });
}

/**
 * Project a point from lat/lon using a bearing (radians) and distance (meters).
 *
 * Think of it like: "I'm standing here, facing this direction, and I walk
 * X meters forward — where do I end up?" Uses spherical Earth math so it
 * works correctly even for long distances (like the 100 km climb segment).
 */
function offsetByBearingMeters(
  lat: number,
  lon: number,
  bearingRad: number,
  distM: number
): { lat: number; lon: number } {
  const R = 6371000; // Earth's radius in meters
  const lat1 = lat * Math.PI / 180;
  const lon1 = lon * Math.PI / 180;
  const angDist = distM / R; // angular distance on the sphere

  const sinLat1 = Math.sin(lat1);
  const cosLat1 = Math.cos(lat1);
  const sinAng = Math.sin(angDist);
  const cosAng = Math.cos(angDist);

  const sinLat2 = sinLat1 * cosAng + cosLat1 * sinAng * Math.cos(bearingRad);
  const lat2 = Math.asin(sinLat2);

  const y = Math.sin(bearingRad) * sinAng * cosLat1;
  const x = cosAng - sinLat1 * sinLat2;
  const lon2 = lon1 + Math.atan2(y, x);

  const outLat = lat2 * 180 / Math.PI;
  const outLon = ((lon2 * 180 / Math.PI) + 540) % 360 - 180;
  return { lat: outLat, lon: outLon };
}

// ─── Variable-Speed Timing ──────────────────────────────────
// These functions handle the tricky part where the plane changes speed
// at different altitudes. We can't just do distance/speed because the
// speed keeps changing as the plane climbs or descends.

/**
 * Compute total time (seconds) to traverse a path with altitude-dependent speed.
 *
 * This is basically numerical integration (like from calculus class).
 * We split the path into tiny segments, figure out the speed at each one,
 * and sum up all the little time = distance/speed values.
 *
 * The easeFn parameter lets us control how the altitude changes — linear,
 * easeOut, etc. This affects the speed profile and therefore the total time.
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
    const f0 = i / segments;
    const f1 = (i + 1) / segments;
    const fMid = (f0 + f1) / 2;

    const alt0 = lerp(startAlt, endAlt, easeFn(f0));
    const alt1 = lerp(startAlt, endAlt, easeFn(f1));
    const altMid = lerp(startAlt, endAlt, easeFn(fMid));

    // Pythagorean theorem: actual segment distance includes altitude change
    const segHorizDist = horizDistM / segments;
    const altChange = alt1 - alt0;
    const segDist = Math.sqrt(segHorizDist * segHorizDist + altChange * altChange);

    const speed = getSpeedForAltitude(altMid, groundElev);
    totalTime += segDist / speed;
  }
  return totalTime;
}

/**
 * For a linear speed ramp (v0 → v1) over a distance, compute the fraction
 * of total time elapsed at a given fraction of total distance.
 *
 * This is the analytical solution — basically solving the integral of
 * 1/v(x) dx where v changes linearly. Way more accurate than just
 * dividing distance by average speed.
 *
 * Formula: timeFrac = ln(v(f)/v0) / ln(v1/v0)
 */
function linearSpeedTimeFrac(distFrac: number, v0: number, v1: number): number {
  if (Math.abs(v1 - v0) < 0.01) return distFrac; // constant speed — simple case
  const vAtF = v0 + (v1 - v0) * distFrac;
  return Math.log(vAtF / v0) / Math.log(v1 / v0);
}

/**
 * Total time to traverse a distance with linearly-changing speed v0 → v1.
 * Also analytical — T = D · ln(v1/v0) / (v1 − v0).
 */
function linearSpeedTotalTime(dist: number, v0: number, v1: number): number {
  if (Math.abs(v1 - v0) < 0.01) return dist / v0;
  return dist * Math.log(v1 / v0) / (v1 - v0);
}

// ─── Plan Flight ────────────────────────────────────────────

/**
 * Calculate flight timing plan (used for preview before starting).
 *
 * This is like a "dry run" — we figure out how long each phase will take
 * BEFORE we actually animate anything. The UI uses this to show the
 * estimated flight duration. Then createFlight() uses the same plan
 * to build the actual 3D path with position samples.
 *
 * Phases: gate → taxi → runway hold → takeoff → climb → cruise →
 *         descent → landing → rollout → arrival taxi → arrival gate
 */
export function planFlight(departure: Airport, destination: Airport): FlightPlan {
  const distanceKm = getDistanceKm(departure, destination);
  const route = DEPARTURE_ROUTES[departure.code] ?? null;
  const arrival = getArrivalRoute(destination.code);

  const elev = departure.elevation;
  const destElev = destination.elevation;

  // ── Gate & hold (fixed) ──────────────────────────────────
  const gateTime = route.gateHoldTime;
  const runwayHoldTime = route.runwayHoldTime;

  // ── Taxi (constant speed: 50 km/h) ──────────────────────
  const taxiPath = [route.gate, ...route.taxiWaypoints, route.runwayThreshold];
  const taxiDistM = computePathDistance(taxiPath);
  const taxiTime = taxiDistM / MAX_GROUND_SPEED_MS;

  // ── Takeoff roll (acceleration 0 → 300 km/h) ───────────
  const rwyPath = [route.runwayThreshold, ...route.runwayWaypoints, route.liftoffPoint];
  const rwyDistM = computePathDistance(rwyPath);
  // Constant acceleration: time = 2 · distance / final_speed
  const takeoffTime = 2 * rwyDistM / LIFTOFF_SPEED_MS;

  // ── Climb geometry ──────────────────────────────────────
  const prevPoint = route.runwayWaypoints[route.runwayWaypoints.length - 1] ?? route.runwayThreshold;
  const runwayHeadingRad = computeBearingRad(prevPoint, route.liftoffPoint);

  const climbExit = offsetByBearingMeters(
    route.liftoffPoint.lat,
    route.liftoffPoint.lon,
    runwayHeadingRad,
    CLIMB_DISTANCE_M
  );
  const climbExitLon = climbExit.lon;
  const climbExitLat = climbExit.lat;

  const climbHorizDistM = haversineDistMeters(
    route.liftoffPoint.lat, route.liftoffPoint.lon,
    climbExitLat, climbExitLon
  );

  const climbStartAlt = elev + 50;
  // Climb all the way to cruise altitude (10 000 m) instead of stopping at 3000m AGL.
  // This gives a much more realistic and gradual climb angle.
  const climbEndAlt = CRUISE_ALTITUDE;

  const climbTime = computeVariableSpeedDuration(
    climbHorizDistM, climbStartAlt, climbEndAlt, elev
  );

  // ── Approach geometry ───────────────────────────────────
  const landingTargetLat = arrival!.touchdownPoint.lat;
  const landingTargetLon = arrival!.touchdownPoint.lon;

  const destHeadingRad = arrival
    ? computeBearingRad(
        arrival.rolloutWaypoints[0] ?? arrival.runwayEnd,
        arrival.touchdownPoint
      )
    : Cesium.Math.toRadians((destination.runwayHeading + 180) % 360);

  const approachEntryLon = landingTargetLon + Math.sin(destHeadingRad) * APPROACH_DISTANCE_DEG;
  const approachEntryLat = landingTargetLat + Math.cos(destHeadingRad) * APPROACH_DISTANCE_DEG;

  const approachHorizDistM = haversineDistMeters(
    approachEntryLat, approachEntryLon,
    landingTargetLat, landingTargetLon
  );

  // Approach starts from cruise altitude — mirrors the climb
  const approachStartAlt = CRUISE_ALTITUDE;
  const approachEndAlt = destElev;

  const landingTime = computeVariableSpeedDuration(
    approachHorizDistM, approachStartAlt, approachEndAlt, destElev, 100, easeOutQuad
  );

  // ── Cruise & Descent ────────────────────────────────────
  // The middle part of the flight — plane is already at cruise altitude,
  // just flying in a straight(ish) great-circle line at full speed.
  const gcDistM = haversineDistMeters(
    climbExitLat, climbExitLon,
    approachEntryLat, approachEntryLon
  );
  const cruiseDistM = gcDistM * 0.8;
  const descentDistM = gcDistM * 0.2;

  // Cruise — already at CRUISE_ALTITUDE, constant MAX_SPEED
  const cruiseTime = Math.max(MIN_CRUISE_DURATION, cruiseDistM / MAX_SPEED_MS);

  // Descent (CRUISE_ALTITUDE → approachStartAlt) — all at MAX_SPEED
  const descentTime = descentDistM / MAX_SPEED_MS;

  // ── Arrival phases ──────────────────────────────────────
  let rolloutTime = 0, arrivalTaxiTime = 0, arrivalGateTime = 0;

  if (arrival) {
    // Rollout: decelerate from ~300 km/h to ~50 km/h
    const rolloutPath = [arrival.touchdownPoint, ...arrival.rolloutWaypoints, arrival.runwayEnd];
    const rolloutDistM = computePathDistance(rolloutPath);
    rolloutTime = linearSpeedTotalTime(rolloutDistM, LIFTOFF_SPEED_MS, MAX_GROUND_SPEED_MS);

    // Arrival taxi: constant 50 km/h
    const arrTaxiPath = [arrival.runwayEnd, ...arrival.taxiToGateWaypoints, arrival.gate];
    const arrTaxiDistM = computePathDistance(arrTaxiPath);
    arrivalTaxiTime = arrTaxiDistM / MAX_GROUND_SPEED_MS;

    arrivalGateTime = ARRIVAL_GATE_HOLD;
  }

  const timing: FlightTiming = {
    gate: gateTime,
    taxi: taxiTime,
    runwayHold: runwayHoldTime,
    takeoff: takeoffTime,
    climb: climbTime,
    cruise: cruiseTime,
    descent: descentTime,
    landing: landingTime,
    rollout: rolloutTime,
    arrivalTaxi: arrivalTaxiTime,
    arrivalGate: arrivalGateTime,
  };

  const totalDuration = timing.gate + timing.taxi + timing.runwayHold +
    timing.takeoff + timing.climb + timing.cruise + timing.descent +
    timing.landing + timing.rollout + timing.arrivalTaxi + timing.arrivalGate;

  return { departure, destination, distanceKm, departureRoute: route, arrivalRoute: arrival, timing, totalDuration };
}

// ─── Create Flight ──────────────────────────────────────────

/**
 * Build the complete flight entity with sampled positions.
 *
 * This is the big one — it takes the flight plan and turns it into an
 * actual animated 3D entity in Cesium. It generates thousands of position
 * samples (lat/lon/alt at specific times) which Cesium then interpolates
 * between to create smooth movement.
 *
 * Each flight phase adds its own samples, and the timing boundaries get
 * updated as we go (because the actual simulation might take slightly
 * different time than what planFlight() estimated).
 */
export function createFlight(
  viewer: Cesium.Viewer,
  plan: FlightPlan
): FlightResult {
  const { departure, destination, timing, totalDuration, departureRoute, arrivalRoute } = plan;

  const start = viewer.clock.currentTime.clone();
  // We'll set the real stopTime later after all phases are built,
  // because the actual duration can differ from the estimate.
  let stop = Cesium.JulianDate.addSeconds(start, totalDuration, new Cesium.JulianDate());

  // Configure clock — CLAMPED stops the clock at stopTime so the flight
  // doesn't loop. The stopTime is recalculated later to match actual duration.
  viewer.clock.startTime = start.clone();
  viewer.clock.stopTime = stop.clone();
  viewer.clock.currentTime = start.clone();
  viewer.clock.clockRange = Cesium.ClockRange.CLAMPED;

  // Build sampled position
  const position = new Cesium.SampledPositionProperty();
  position.setInterpolationOptions({
  interpolationDegree: 3,
  interpolationAlgorithm: Cesium.HermitePolynomialApproximation,
});

  const elev = departure.elevation;
  const destElev = destination.elevation;

  // Phase time boundaries
  const t_gateEnd = timing.gate;
  const t_taxiEnd = t_gateEnd + timing.taxi;
  const t_holdEnd = t_taxiEnd + timing.runwayHold;
  const t_takeoffEnd = t_holdEnd + timing.takeoff;
  let t_climbEnd = t_takeoffEnd + timing.climb;        // updated after climb sim
  let t_cruiseEnd = t_climbEnd + timing.cruise;
  let t_descentEnd = t_cruiseEnd + timing.descent;
  let t_landingEnd = t_descentEnd + timing.landing;    // updated after landing sim
  let t_rolloutEnd = t_landingEnd + timing.rollout;
  let t_arrivalTaxiEnd = t_rolloutEnd + timing.arrivalTaxi;
  // t_arrivalGateEnd = totalDuration

  // ── Determine liftoff position & runway heading ────────
  let liftoffLat: number;
  let liftoffLon: number;
  let runwayHeadingRad: number;

  if (departureRoute) {
    liftoffLat = departureRoute.liftoffPoint.lat;
    liftoffLon = departureRoute.liftoffPoint.lon;
    const prevPoint = departureRoute.runwayWaypoints[departureRoute.runwayWaypoints.length - 1] ?? departureRoute.runwayThreshold;
    runwayHeadingRad = computeBearingRad(prevPoint, departureRoute.liftoffPoint);
  } else {
    runwayHeadingRad = Cesium.Math.toRadians(departure.runwayHeading);
    liftoffLon = departure.lon + Math.sin(runwayHeadingRad) * 0.02;
    liftoffLat = departure.lat + Math.cos(runwayHeadingRad) * 0.02;
  }

  // Climb goes all the way to cruise altitude now (not just 3000m AGL)
  const climbStartAlt = elev + 50;
  const climbEndAlt = CRUISE_ALTITUDE;
  const approachEntryAlt = CRUISE_ALTITUDE;

  // Determine landing target
  let landingTargetLat: number;
  let landingTargetLon: number;
  let landingTargetElev: number;

  landingTargetLat = arrivalRoute!.touchdownPoint.lat;
  landingTargetLon = arrivalRoute!.touchdownPoint.lon;
  landingTargetElev = destElev;

  // Approach entry point
  const destHeadingRad = arrivalRoute
    ? computeBearingRad(arrivalRoute.rolloutWaypoints[0] ?? arrivalRoute.runwayEnd, arrivalRoute.touchdownPoint)
    : Cesium.Math.toRadians((destination.runwayHeading + 180) % 360);

  const approachEntryLon = landingTargetLon + Math.sin(destHeadingRad) * APPROACH_DISTANCE_DEG;
  const approachEntryLat = landingTargetLat + Math.cos(destHeadingRad) * APPROACH_DISTANCE_DEG;

  // Approach entry cartographic — the ILS cone entry point (~27 km before touchdown)
  const approachEntryCartographic = Cesium.Cartographic.fromDegrees(approachEntryLon, approachEntryLat);

  function addSample(elapsed: number, lon: number, lat: number, alt: number): void {
    const time = Cesium.JulianDate.addSeconds(start, elapsed, new Cesium.JulianDate());
    position.addSample(time, Cesium.Cartesian3.fromDegrees(lon, lat, alt));
  }

  // ── ROUTE-BASED DEPARTURE ────────────────────────────────
  buildRouteDeparture(departureRoute!, elev, addSample, timing);

  // ── CLIMB (rate-limited turn, 3°/s Standard Rate Turn) ───
  // Steers toward approachEntry each step; the aircraft naturally sweeps a
  // wide arc without any sudden course change after liftoff.
  const climbResult = buildClimbWithTurn(
    liftoffLon, liftoffLat,
    runwayHeadingRad,
    elev,
    climbStartAlt, climbEndAlt,
    approachEntryLon, approachEntryLat,
    t_takeoffEnd,
    timing.climb,
    addSample
  );
  // Synchronise all downstream phase boundaries with actual climb duration
  t_climbEnd = t_takeoffEnd + climbResult.actualDuration;
  t_cruiseEnd = t_climbEnd + timing.cruise;
  t_descentEnd = t_cruiseEnd + timing.descent;
  t_landingEnd = t_descentEnd + timing.landing;
  t_rolloutEnd = t_landingEnd + timing.rollout;
  t_arrivalTaxiEnd = t_rolloutEnd + timing.arrivalTaxi;

  // Great circle from actual climb-end position to approach entry
  const climbEndCartographic = Cesium.Cartographic.fromDegrees(climbResult.endLon, climbResult.endLat);
  const geodesic = new Cesium.EllipsoidGeodesic(climbEndCartographic, approachEntryCartographic);

  // ── CRUISE (great circle at cruise altitude) ──────────────
  // The plane is already at CRUISE_ALTITUDE after the climb phase,
  // so we just fly level at constant altitude and max speed.
  // No more need for the CRUISE_CLIMB_FRAC hack since climb now goes
  // all the way to 10 000 m.
  const cruiseSamples = 1000;
  for (let i = 0; i <= cruiseSamples; i++) {
    const frac = i / cruiseSamples;
    const elapsed = t_climbEnd + frac * timing.cruise;

    const gcFrac = lerp(0.0, 0.80, frac); // covers 0–80% of geodesic
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);

    addSample(elapsed, lon, lat, CRUISE_ALTITUDE);
  }

  // ── DESCENT (cruise altitude → approach altitude) ─────────
  const descentSamples = 200;
  for (let i = 0; i <= descentSamples; i++) {
    const frac = i / descentSamples;
    const elapsed = t_cruiseEnd + frac * timing.descent;

    // Covers the last 20% of the geodesic (80% → 100%), ending at approachEntry
    const gcFrac = lerp(0.80, 1.0, frac);
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);
    const alt = lerp(CRUISE_ALTITUDE, approachEntryAlt, easeInOutCubic(frac));

    addSample(elapsed, lon, lat, alt);
  }

  // Geodesic heading at approachEntry — tangent direction at 99.9% → 100%
  const gc999 = geodesic.interpolateUsingFraction(0.999);
  const gc100 = geodesic.interpolateUsingFraction(1.0);
  const approachInitialHeading = computeBearingRad(
    { lat: Cesium.Math.toDegrees(gc999.latitude), lon: Cesium.Math.toDegrees(gc999.longitude) },
    { lat: Cesium.Math.toDegrees(gc100.latitude),  lon: Cesium.Math.toDegrees(gc100.longitude)  }
  );

  // ── LANDING (rate-limited turn, 3°/s Standard Rate Turn) ─
  // Starts at approachEntry with the geodesic arrival heading and turns
  // smoothly onto the runway centerline — no speed penalty.
  const landingResult = buildApproachWithTurn(
    approachEntryLon, approachEntryLat,
    approachEntryAlt,
    approachInitialHeading,
    landingTargetLon, landingTargetLat,
    landingTargetElev,
    t_descentEnd,
    timing.landing,
    addSample
  );
  // Update landing boundary with actual approach simulation duration
  t_landingEnd = t_descentEnd + landingResult.actualDuration;
  t_rolloutEnd = t_landingEnd + timing.rollout;
  t_arrivalTaxiEnd = t_rolloutEnd + timing.arrivalTaxi;

  // ── ARRIVAL (rollout → taxi → gate) ──────────────────────
  if (arrivalRoute) {
    buildRouteArrival(arrivalRoute, destElev, addSample, timing, t_landingEnd);
  }

  // ── FIX: Recalculate the actual total duration after all simulations ──
  // The climb and landing simulations can produce slightly different durations
  // than what planFlight() estimated. If we don't update stopTime, the clock
  // will either freeze too early or leave a gap at the end.
  const actualTotalDuration = t_arrivalTaxiEnd + timing.arrivalGate;
  stop = Cesium.JulianDate.addSeconds(start, actualTotalDuration, new Cesium.JulianDate());
  viewer.clock.stopTime = stop.clone();

  /**
   * Figure out which phase of the flight we're currently in.
   * This is called every 200ms from main.ts to update the UI label
   * ("Climbing", "Cruising", etc.) and also to trigger arrival handling.
   *
   * It just checks elapsed time against the phase boundaries we computed.
   * Uses the ACTUAL boundaries (after simulation adjustments), not the
   * original estimates — that's why the totalDuration reference was
   * changed to actualTotalDuration.
   */
  function getPhase(currentTime: Cesium.JulianDate): FlightPhase {
    const elapsed = Cesium.JulianDate.secondsDifference(currentTime, start);
    if (elapsed < 0) return "preflight";
    if (timing.gate > 0 && elapsed <= t_gateEnd) return "gate";
    if (timing.taxi > 0 && elapsed <= t_taxiEnd) return "taxi";
    if (timing.runwayHold > 0 && elapsed <= t_holdEnd) return "runway_hold";
    if (elapsed <= t_takeoffEnd) return "takeoff";
    if (elapsed <= t_climbEnd) return "climb";
    if (elapsed <= t_cruiseEnd) return "cruise";
    if (elapsed <= t_descentEnd) return "descent";
    if (elapsed <= t_landingEnd) return "landing";
    if (timing.rollout > 0 && elapsed <= t_rolloutEnd) return "rollout";
    if (timing.arrivalTaxi > 0 && elapsed <= t_arrivalTaxiEnd) return "arrival_taxi";
    // Small epsilon (0.5s) so floating-point rounding at CLAMPED stopTime
    // doesn't keep us stuck in "arrival_gate" forever
    if (timing.arrivalGate > 0 && elapsed < actualTotalDuration - 0.5) return "arrival_gate";
    return "arrived";
  }

  const FLARE_START_DIST = 1500; // meters before touchdown to begin pitch-up
  const FLARE_PITCH_RAD = Cesium.Math.toRadians(18);
  const BANK_MAX_RAD = Cesium.Math.toRadians(25);
  const BANK_TURN_RATE = Cesium.Math.toRadians(6); // rad/s for full bank
  const BANK_DT = 0.5; // seconds ahead for turn rate estimation

  const velocityOrientation = new Cesium.VelocityOrientationProperty(position);
  const scratchMatrix = new Cesium.Matrix3();
  const scratchAxis = new Cesium.Cartesian3();
  const scratchPitchQuat = new Cesium.Quaternion();

  const orientation = new Cesium.CallbackProperty((time, result) => {
    if (!time) return result;
    const curr = position.getValue(time);
    if (!curr) return result;


    const base = velocityOrientation.getValue(time, result);
    if (!base) return result;

    let pitch = 0;
    let roll = 0;
    const phase = getPhase(time);

    if (phase === "landing" || phase === "rollout") {
      const currCarto = Cesium.Cartographic.fromCartesian(curr);
      const distToTouchdown = haversineDistMeters(
        Cesium.Math.toDegrees(currCarto.latitude),
        Cesium.Math.toDegrees(currCarto.longitude),
        landingTargetLat,
        landingTargetLon
      );
      if (distToTouchdown <= FLARE_START_DIST) {
        const t = 1 - Math.max(0, Math.min(1, distToTouchdown / FLARE_START_DIST));
        pitch = -lerp(0, FLARE_PITCH_RAD, easeOutQuad(t));
      }
    }

    const allowBank = phase === "climb" || phase === "cruise" || phase === "descent" || phase === "landing";
    const nextTime = Cesium.JulianDate.addSeconds(time, BANK_DT, new Cesium.JulianDate());
    const nextTime2 = Cesium.JulianDate.addSeconds(nextTime, BANK_DT, new Cesium.JulianDate());
    const nextPos = position.getValue(nextTime);
    const nextPos2 = position.getValue(nextTime2);
    if (allowBank && nextPos && nextPos2) {
      const currCarto = Cesium.Cartographic.fromCartesian(curr);
      const nextCarto = Cesium.Cartographic.fromCartesian(nextPos);
      const nextCarto2 = Cesium.Cartographic.fromCartesian(nextPos2);

      const headingNow = computeBearingRad(
        { lat: Cesium.Math.toDegrees(currCarto.latitude), lon: Cesium.Math.toDegrees(currCarto.longitude) },
        { lat: Cesium.Math.toDegrees(nextCarto.latitude), lon: Cesium.Math.toDegrees(nextCarto.longitude) }
      );
      const headingNext = computeBearingRad(
        { lat: Cesium.Math.toDegrees(nextCarto.latitude), lon: Cesium.Math.toDegrees(nextCarto.longitude) },
        { lat: Cesium.Math.toDegrees(nextCarto2.latitude), lon: Cesium.Math.toDegrees(nextCarto2.longitude) }
      );
      const dHead = Math.atan2(Math.sin(headingNext - headingNow), Math.cos(headingNext - headingNow));
      const turnRate = dHead / BANK_DT;
      const bankFrac = Math.max(-1, Math.min(1, turnRate / BANK_TURN_RATE));
      roll = BANK_MAX_RAD * bankFrac;
    }

    if (pitch === 0 && roll === 0) return base;

    Cesium.Matrix3.fromQuaternion(base, scratchMatrix);
    Cesium.Matrix3.getColumn(scratchMatrix, 1, scratchAxis); // local right axis (model Y)
    Cesium.Quaternion.fromAxisAngle(scratchAxis, pitch, scratchPitchQuat);
    let adjusted = Cesium.Quaternion.multiply(scratchPitchQuat, base, result);

    if (roll !== 0) {
      Cesium.Matrix3.fromQuaternion(adjusted, scratchMatrix);
      Cesium.Matrix3.getColumn(scratchMatrix, 0, scratchAxis); // local forward axis
      const rollQuat = Cesium.Quaternion.fromAxisAngle(scratchAxis, roll, new Cesium.Quaternion());
      adjusted = Cesium.Quaternion.multiply(rollQuat, adjusted, result);
    }

    return adjusted;
  }, false);

  // Create the 3D entity that Cesium will render and track with the camera.
  // The availability interval tells Cesium when this entity "exists" —
  // we use the updated stop time so it covers the full actual flight.
  const entity = viewer.entities.add({
    name: `Flight ${departure.code} → ${destination.code}`,
    availability: new Cesium.TimeIntervalCollection([
      new Cesium.TimeInterval({ start, stop }),
    ]),
    position: position,
    orientation: orientation,
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
      trailTime: actualTotalDuration,
    },
  });

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

// This type is just a shorthand for the "add a position sample" callback.
// Every phase builder uses it to register (time, lon, lat, alt) points.
type AddSampleFn = (elapsed: number, lon: number, lat: number, alt: number) => void;

/**
 * Build samples for a route-based departure: gate → taxi → hold → takeoff roll → liftoff.
 *
 * This handles everything that happens on the ground before the plane
 * actually takes off. The plane sits at the gate, taxis to the runway,
 * waits for clearance, then accelerates down the runway and lifts off.
 *
 * Taxi speed: 50 km/h (constant)
 * Takeoff roll: 0 → 300 km/h (constant acceleration, like v = a*t)
 */
function buildRouteDeparture(
  route: DepartureRoute,
  elevation: number,
  addSample: AddSampleFn,
  timing: FlightTiming
): void {
  let t = 0;

  // ── GATE HOLD ──────────────────────────────────────────
  // Add samples with a tiny drift so VelocityOrientationProperty can compute heading
  const firstTarget = route.taxiWaypoints[0] ?? route.runwayThreshold;
  const gateBearing = computeBearingRad(route.gate, firstTarget);
  const DRIFT = 0.0000005; // imperceptible drift per second

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

  // ── TAXI (constant speed: 50 km/h → time proportional to distance) ──
  const taxiPath: RouteWaypoint[] = [
    route.gate,
    ...route.taxiWaypoints,
    route.runwayThreshold,
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
      const elapsed = t + timeFrac * timing.taxi;
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);
      addSample(elapsed, lon, lat, elevation);
    }
    taxiCumDist += segDist;
  }
  t += timing.taxi;

  // ── TAXI→RUNWAY ALIGNMENT ──────────────────────────────
  // The taxi arrives from a random direction, but the runway hold must
  // face down the runway. With Hermite interpolation the velocity vector
  // is derived from the curve tangent, so without these extra samples
  // the plane would spin around trying to find the right heading.
  // We add a few tightly-spaced "alignment" samples that drift along
  // the runway bearing — this overwrites the incoming taxi tangent.
  const firstRwyTarget = route.runwayWaypoints[0] ?? route.liftoffPoint;
  const rwyBearing = computeBearingRad(route.runwayThreshold, firstRwyTarget);

  const ALIGN_DURATION = 0.3; // seconds — very short, almost instant
  const ALIGN_STEPS = 4;      // enough samples to dominate the Hermite tangent
  for (let i = 0; i <= ALIGN_STEPS; i++) {
    const frac = i / ALIGN_STEPS;
    const elapsed = t + frac * ALIGN_DURATION;
    addSample(
      elapsed,
      route.runwayThreshold.lon + Math.sin(rwyBearing) * DRIFT * (frac * ALIGN_DURATION),
      route.runwayThreshold.lat + Math.cos(rwyBearing) * DRIFT * (frac * ALIGN_DURATION),
      elevation
    );
  }
  t += ALIGN_DURATION;

  // ── RUNWAY HOLD ────────────────────────────────────────
  // Plane is already facing the runway thanks to alignment samples above.
  // Continue drifting in the same direction during the hold.
  for (let i = 0; i <= 3; i++) {
    const frac = i / 3;
    const elapsed = t + frac * timing.runwayHold;
    addSample(
      elapsed,
      route.runwayThreshold.lon + Math.sin(rwyBearing) * DRIFT * (ALIGN_DURATION + frac * timing.runwayHold),
      route.runwayThreshold.lat + Math.cos(rwyBearing) * DRIFT * (ALIGN_DURATION + frac * timing.runwayHold),
      elevation
    );
  }
  t += timing.runwayHold;

  // ── TAKEOFF ROLL (acceleration 0 → 300 km/h) ──────────
  // For constant acceleration from rest: timeFrac = sqrt(distFrac)
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

      // Constant acceleration from rest: time = totalTime · √(distFrac)
      const elapsed = t + timing.takeoff * Math.sqrt(distFrac);

      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);

      // Start climbing in the last segment (liftoff)
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
 * Build samples for a route-based arrival: rollout → taxi to gate → gate hold.
 *
 * This is basically the departure in reverse — after touchdown the plane
 * decelerates on the runway (rollout), turns off the runway, and taxis
 * to the destination gate at low speed.
 *
 * Rollout: decelerates from 300 → 50 km/h (reverse of takeoff acceleration)
 * Taxi: constant 50 km/h to the gate
 */
function buildRouteArrival(
  route: ArrivalRoute,
  elevation: number,
  addSample: AddSampleFn,
  timing: FlightTiming,
  tStart: number
): void {
  let t = tStart;

  // ── ROLLOUT (deceleration: 300 → 50 km/h) ────────────
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

  let rollCumDist = 0;
  for (let seg = 0; seg < rolloutPath.length - 1; seg++) {
    const from = rolloutPath[seg];
    const to = rolloutPath[seg + 1];
    const segDist = rollSegDists[seg];

    const subStart = seg === 0 ? 0 : 1;
    for (let sub = subStart; sub <= 4; sub++) {
      const frac = sub / 4;
      const distAtSub = rollCumDist + frac * segDist;
      const distFrac = totalRollDist > 0 ? distAtSub / totalRollDist : 0;

      // Linear deceleration: use analytical time fraction
      const timeFrac = linearSpeedTimeFrac(distFrac, LIFTOFF_SPEED_MS, MAX_GROUND_SPEED_MS);
      const elapsed = t + timeFrac * timing.rollout;

      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);
      addSample(elapsed, lon, lat, elevation);
    }
    rollCumDist += segDist;
  }
  t += timing.rollout;

  // ── ARRIVAL TAXI (constant speed: 50 km/h) ────────────
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

  // ── ARRIVAL GATE HOLD ──────────────────────────────────
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


// ─── Math Utility Functions ─────────────────────────────────
// These are small helper functions used all over the place.
// lerp = linear interpolation, easing functions control acceleration curves.

/** Linear interpolation — blends between a and b based on t (0 to 1). */
function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

/** Ease-in (starts slow, ends fast) — used for takeoff acceleration feel. */
function easeInQuad(t: number): number {
  return t * t;
}

/** Ease-out (starts fast, ends slow) — used for landing deceleration feel. */
function easeOutQuad(t: number): number {
  return t * (2 - t);
}

/** Ease-in-out (smooth start AND end) — used for the descent altitude curve. */
function easeInOutCubic(t: number): number {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

/**
 * Compute the initial bearing (in radians) from one waypoint to another.
 * North = 0, East = PI/2, South = PI, West = 3PI/2.
 * This is essential for figuring out which direction the plane should face.
 */
function computeBearingRad(from: RouteWaypoint, to: RouteWaypoint): number {
  const dLon = (to.lon - from.lon) * Math.PI / 180;
  const lat1 = from.lat * Math.PI / 180;
  const lat2 = to.lat * Math.PI / 180;
  const y = Math.sin(dLon) * Math.cos(lat2);
  const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
  return (Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI);
}

// ─── Rate-Limited Turn Simulation ───────────────────────────

interface TurnSimResult {
  endLon: number;
  endLat: number;
  endAlt: number;
  endHeading: number;
  actualDuration: number;
}

const SIM_DT = 0.5;                                     // seconds per simulation step
const MAX_TURN_RATE_RAD = Cesium.Math.toRadians(3);     // 3°/s — Standard Rate Turn

/**
 * Clamp a heading change to ±maxStep using shortest-path normalization.
 *
 * Without this, the plane might try to turn 350° to the right instead
 * of 10° to the left. The atan2 trick normalizes the angle difference
 * to [-PI, PI] so it always picks the shorter turn direction.
 */
function clampAngleChange(current: number, target: number, maxStep: number): number {
  let diff = target - current;
  // Normalize to [-PI, PI] — this is the key trick for shortest-path turns
  diff = Math.atan2(Math.sin(diff), Math.cos(diff));
  if (Math.abs(diff) <= maxStep) return target;
  return current + Math.sign(diff) * maxStep;
}

/**
 * Move a lat/lon position `distMeters` in direction `headingRad`.
 *
 * Similar to offsetByBearingMeters but slightly simpler — used in the
 * step-by-step simulation loops (climb and approach) where we advance
 * the plane's position each SIM_DT step.
 */
function moveWithHeading(
  lon: number, lat: number,
  headingRad: number, distMeters: number
): { lon: number; lat: number } {
  const R = 6371000;
  const latRad = lat * Math.PI / 180;
  const lonRad = lon * Math.PI / 180;
  const angDist = distMeters / R;
  const newLat = Math.asin(
    Math.sin(latRad) * Math.cos(angDist) +
    Math.cos(latRad) * Math.sin(angDist) * Math.cos(headingRad)
  );
  const newLon = lonRad + Math.atan2(
    Math.sin(headingRad) * Math.sin(angDist) * Math.cos(latRad),
    Math.cos(angDist) - Math.sin(latRad) * Math.sin(newLat)
  );
  return {
    lon: (newLon * 180 / Math.PI + 540) % 360 - 180,
    lat: newLat * 180 / Math.PI,
  };
}

/**
 * Simulate the climb phase with a rate-limited heading (3°/s).
 *
 * This is one of the coolest parts — instead of just teleporting the plane
 * to cruise altitude, we simulate it step-by-step. Every 0.5 seconds:
 *   1. Figure out which direction the cruise waypoint is
 *   2. Turn the plane toward it (but max 3°/s, like a real plane)
 *   3. Move forward based on current speed (which depends on altitude)
 *   4. Increase altitude gradually
 *
 * The result is a nice smooth arc from the runway heading to the
 * cruise course, which looks way more realistic than an instant turn.
 *
 * The plane now climbs all the way to 10 000 m (cruise altitude)
 * over about 100 km, giving a reasonable ~5.7° climb angle.
 */
function buildClimbWithTurn(
  liftoffLon: number, liftoffLat: number,
  runwayHeadingRad: number,
  elev: number,
  climbStartAlt: number, climbEndAlt: number,
  cruiseTargetLon: number, cruiseTargetLat: number,
  t_takeoffEnd: number,
  climbDuration: number,
  addSample: AddSampleFn
): TurnSimResult {
  let currentLon = liftoffLon;
  let currentLat = liftoffLat;
  let currentHeading = runwayHeadingRad;
  let currentAlt = climbStartAlt;
  const baseMaxStep = MAX_TURN_RATE_RAD * SIM_DT;

  // First sample: right at liftoff position
  addSample(t_takeoffEnd, currentLon, currentLat, currentAlt);

  let elapsed = 0;
  while (currentAlt < climbEndAlt - 0.5) {
    elapsed += SIM_DT;

    // Ease-in the turn rate so the plane flies straight right after
    // takeoff and gradually starts turning toward the cruise waypoint.
    const turnFrac = climbDuration > 0 ? Math.min(elapsed / climbDuration, 1.0) : 1.0;
    const maxStep = baseMaxStep * easeInQuad(turnFrac);

    // Figure out which way the cruise target is and turn toward it
    const targetHeading = computeBearingRad(
      { lat: currentLat, lon: currentLon },
      { lat: cruiseTargetLat, lon: cruiseTargetLon }
    );
    currentHeading = clampAngleChange(currentHeading, targetHeading, maxStep);

    // Move forward at the speed appropriate for current altitude
    const speed = getSpeedForAltitude(currentAlt, elev);
    const next = moveWithHeading(currentLon, currentLat, currentHeading, speed * SIM_DT);
    currentLon = next.lon;
    currentLat = next.lat;

    // Altitude increases linearly with time
    const altFrac = Math.min(elapsed / climbDuration, 1.0);
    currentAlt = lerp(climbStartAlt, climbEndAlt, altFrac);

    addSample(t_takeoffEnd + elapsed, currentLon, currentLat, currentAlt);

    if (elapsed > climbDuration * 2) break; // safety guard against infinite loops
  }

  return {
    endLon: currentLon, endLat: currentLat,
    endAlt: currentAlt, endHeading: currentHeading,
    actualDuration: elapsed,
  };
}

/**
 * Simulate the approach/landing phase with a rate-limited heading (3°/s).
 *
 * This is the mirror image of buildClimbWithTurn — the plane starts
 * at cruise altitude (~100 km from the runway) and gradually descends
 * while turning to align with the runway centerline.
 *
 * The altitude uses easeOutQuad (drops quickly at first, then levels off
 * near the ground) which mimics a real ILS approach glide slope.
 * The plane slows down naturally as it gets lower because
 * getSpeedForAltitude() returns lower speeds at lower altitudes.
 */
function buildApproachWithTurn(
  approachEntryLon: number, approachEntryLat: number,
  approachEntryAlt: number,
  initialHeadingRad: number,
  touchdownLon: number, touchdownLat: number,
  destElev: number,
  t_descentEnd: number,
  landingDuration: number,
  addSample: AddSampleFn
): TurnSimResult {
  let currentLon = approachEntryLon;
  let currentLat = approachEntryLat;
  let currentHeading = initialHeadingRad;
  let currentAlt = approachEntryAlt;
  const maxStep = MAX_TURN_RATE_RAD * SIM_DT;
  const totalApproachDist = Math.max(
    1,
    haversineDistMeters(approachEntryLat, approachEntryLon, touchdownLat, touchdownLon)
  );

  addSample(t_descentEnd, currentLon, currentLat, currentAlt);

  let elapsed = 0;
  while (true) {
    elapsed += SIM_DT;

    // Steer toward the touchdown point, limited by max turn rate
    const targetHeading = computeBearingRad(
      { lat: currentLat, lon: currentLon },
      { lat: touchdownLat, lon: touchdownLon }
    );
    currentHeading = clampAngleChange(currentHeading, targetHeading, maxStep);

    // Check if we're close enough to snap to touchdown
    const speed = getSpeedForAltitude(currentAlt, destElev);
    const distToTarget = haversineDistMeters(currentLat, currentLon, touchdownLat, touchdownLon);
    if (distToTarget <= speed * SIM_DT) {
      // We've arrived at the runway — snap to touchdown position
      currentLon = touchdownLon;
      currentLat = touchdownLat;
      currentAlt = destElev;
      addSample(t_descentEnd + elapsed, currentLon, currentLat, currentAlt);
      break;
    }

    // Move forward
    const next = moveWithHeading(currentLon, currentLat, currentHeading, speed * SIM_DT);
    currentLon = next.lon;
    currentLat = next.lat;

    // Descend using easeOutQuad based on how far we still are from touchdown.
    // easeOutQuad makes the plane drop faster when far away and level off
    // as it gets close — this prevents the plane from hitting the ground early.
    const distToTargetAfter = haversineDistMeters(currentLat, currentLon, touchdownLat, touchdownLon);
    const distFrac = Math.min(1.0, Math.max(0.0, 1 - distToTargetAfter / totalApproachDist));
    currentAlt = lerp(approachEntryAlt, destElev, easeOutQuad(distFrac));

    addSample(t_descentEnd + elapsed, currentLon, currentLat, currentAlt);

    if (elapsed > landingDuration * 3) break; // safety guard
  }

  return {
    endLon: currentLon, endLat: currentLat,
    endAlt: destElev, endHeading: currentHeading,
    actualDuration: elapsed,
  };
}
