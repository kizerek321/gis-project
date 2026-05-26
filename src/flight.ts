/**
 * Flight path engine — generates realistic departure → cruise → landing paths.
 *
 * Speed profile (AGL — above ground level):
 *   Ground:    max 50 km/h (taxi, rollout end)
 *   0–400m:    300 km/h (liftoff / final approach)
 *   400–800m:  300 → 450 km/h
 *   800–3000m: 450 → 900 km/h (main speed increase)
 *   3000m+:    900 km/h (cruise, never exceeded)
 *
 * Climb goes STRAIGHT from the runway heading (no turning).
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
const CLIMB_DISTANCE_DEG = 0.1;      // ~11 km horizontal climb distance
const APPROACH_DISTANCE_DEG = 0.25;  // ~27 km approach distance

// Fixed durations
const ARRIVAL_GATE_HOLD = 5;         // seconds parked at destination gate
const MIN_CRUISE_DURATION = 15;      // seconds minimum cruise time

// ─── Speed Profile ──────────────────────────────────────────

/**
 * Returns the airplane speed (m/s) based on altitude above ground.
 *
 * Profile:
 *   Ground level:     50 km/h   (MAX_GROUND_SPEED)
 *   0 – 400m AGL:    300 km/h  (LIFTOFF_SPEED)
 *   400 – 800m AGL:  300→450   (ramp to CLIMB_MID_SPEED)
 *   800 – 3000m AGL: 450→900   (main speed increase)
 *   Above 3000m AGL: 900 km/h  (MAX_SPEED — never exceeded)
 */
function getSpeedForAltitude(altitude: number, groundElev: number): number {
  const agl = altitude - groundElev;
  if (agl <= 0) return MAX_GROUND_SPEED_MS;
  if (agl <= SPEED_RAMP_START_AGL) return LIFTOFF_SPEED_MS;
  if (agl <= SPEED_RAMP_MID_AGL) {
    const frac = (agl - SPEED_RAMP_START_AGL) / (SPEED_RAMP_MID_AGL - SPEED_RAMP_START_AGL);
    return lerp(LIFTOFF_SPEED_MS, CLIMB_MID_SPEED_MS, frac);
  }
  if (agl <= SPEED_RAMP_END_AGL) {
    const frac = (agl - SPEED_RAMP_MID_AGL) / (SPEED_RAMP_END_AGL - SPEED_RAMP_MID_AGL);
    return lerp(CLIMB_MID_SPEED_MS, MAX_SPEED_MS, frac);
  }
  return MAX_SPEED_MS;
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

// Variable-Speed Timing 

/**
 * Compute total time (seconds) to traverse a path with altitude-dependent speed.
 * Uses numerical integration over the given number of segments.
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

    const segHorizDist = horizDistM / segments;
    const altChange = alt1 - alt0;
    const segDist = Math.sqrt(segHorizDist * segHorizDist + altChange * altChange);

    const speed = getSpeedForAltitude(altMid, groundElev);
    totalTime += segDist / speed;
  }
  return totalTime;
}

/**
 * Compute cumulative time at each of `samples+1` evenly-spaced path positions.
 * Returns array of length `samples+1` with cumTimes[0]=0.
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
    const segDist = Math.sqrt(segHorizDist * segHorizDist + altChange * altChange);

    const speed = getSpeedForAltitude(altMid, groundElev);
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

// ─── Plan Flight ────────────────────────────────────────────

/**
 * Calculate flight timing plan (used for preview before starting).
 * Computes phase durations from real waypoint distances and speed profiles.
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

  const climbExitLon = route.liftoffPoint.lon + Math.sin(runwayHeadingRad) * CLIMB_DISTANCE_DEG;
  const climbExitLat = route.liftoffPoint.lat + Math.cos(runwayHeadingRad) * CLIMB_DISTANCE_DEG;

  const climbHorizDistM = haversineDistMeters(
    route.liftoffPoint.lat, route.liftoffPoint.lon,
    climbExitLat, climbExitLon
  );

  const climbStartAlt = elev + 50;
  const climbEndAlt = elev + SPEED_RAMP_END_AGL; // 3000m AGL

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

  const approachStartAlt = destElev + SPEED_RAMP_END_AGL; // 3000m AGL
  const approachEndAlt = destElev;

  const landingTime = computeVariableSpeedDuration(
    approachHorizDistM, approachStartAlt, approachEndAlt, destElev, 100, easeOutQuad
  );

  // ── Cruise & Descent ────────────────────────────────────
  const gcDistM = haversineDistMeters(
    climbExitLat, climbExitLon,
    approachEntryLat, approachEntryLon
  );
  const cruiseDistM = gcDistM * 0.8;
  const descentDistM = gcDistM * 0.2;

  // Cruise (includes climb from climbEndAlt to CRUISE_ALTITUDE) — all at MAX_SPEED
  const cruiseTime = Math.max(MIN_CRUISE_DURATION, cruiseDistM / MAX_SPEED_MS);

  // Descent (CRUISE_ALTITUDE → approachStartAlt) — all at MAX_SPEED (above 3000m AGL)
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
 */
export function createFlight(
  viewer: Cesium.Viewer,
  plan: FlightPlan
): FlightResult {
  const { departure, destination, timing, totalDuration, departureRoute, arrivalRoute } = plan;

  const start = viewer.clock.currentTime.clone();
  const stop = Cesium.JulianDate.addSeconds(start, totalDuration, new Cesium.JulianDate());

  // Configure clock
  viewer.clock.startTime = start.clone();
  viewer.clock.stopTime = stop.clone();
  viewer.clock.currentTime = start.clone();
  viewer.clock.clockRange = Cesium.ClockRange.CLAMPED;

  // Build sampled position
  const position = new Cesium.SampledPositionProperty();
  position.setInterpolationOptions({
    interpolationDegree: 1,
    interpolationAlgorithm: Cesium.LinearApproximation,
  });

  const elev = departure.elevation;
  const destElev = destination.elevation;

  // Phase time boundaries
  const t_gateEnd = timing.gate;
  const t_taxiEnd = t_gateEnd + timing.taxi;
  const t_holdEnd = t_taxiEnd + timing.runwayHold;
  const t_takeoffEnd = t_holdEnd + timing.takeoff;
  const t_climbEnd = t_takeoffEnd + timing.climb;
  const t_cruiseEnd = t_climbEnd + timing.cruise;
  const t_descentEnd = t_cruiseEnd + timing.descent;
  const t_landingEnd = t_descentEnd + timing.landing;
  const t_rolloutEnd = t_landingEnd + timing.rollout;
  const t_arrivalTaxiEnd = t_rolloutEnd + timing.arrivalTaxi;
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

  // Climb exit — straight ahead from runway heading
  const climbExitLon = liftoffLon + Math.sin(runwayHeadingRad) * CLIMB_DISTANCE_DEG;
  const climbExitLat = liftoffLat + Math.cos(runwayHeadingRad) * CLIMB_DISTANCE_DEG;

  // Dynamic altitude thresholds (AGL-based)
  const climbStartAlt = elev + 50;
  const climbEndAlt = elev + SPEED_RAMP_END_AGL;          // 3000m AGL
  const approachEntryAlt = destElev + SPEED_RAMP_END_AGL;  // 3000m AGL

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

  const climbExitCartographic = Cesium.Cartographic.fromDegrees(climbExitLon, climbExitLat);
  const approachEntryCartographic = Cesium.Cartographic.fromDegrees(approachEntryLon, approachEntryLat);

  // Great circle connects the climb exit to the approach entry point
  const geodesic = new Cesium.EllipsoidGeodesic(climbExitCartographic, approachEntryCartographic);

  function addSample(elapsed: number, lon: number, lat: number, alt: number): void {
    const time = Cesium.JulianDate.addSeconds(start, elapsed, new Cesium.JulianDate());
    position.addSample(time, Cesium.Cartesian3.fromDegrees(lon, lat, alt));
  }

  // ── ROUTE-BASED DEPARTURE ────────────────────────────────
  buildRouteDeparture(departureRoute!, elev, addSample, timing);

  // ── CLIMB (liftoff → 3000m AGL, straight from runway heading) ──
  const climbSamples = 100;
  const climbHorizDistM = haversineDistMeters(liftoffLat, liftoffLon, climbExitLat, climbExitLon);
  const climbCumTimes = computeCumulativeTimes(
    climbHorizDistM, climbStartAlt, climbEndAlt, elev, climbSamples
  );

  for (let i = 0; i <= climbSamples; i++) {
    const frac = i / climbSamples;
    const elapsed = t_takeoffEnd + climbCumTimes[i];
    const lon = lerp(liftoffLon, climbExitLon, frac);
    const lat = lerp(liftoffLat, climbExitLat, frac);
    const alt = lerp(climbStartAlt, climbEndAlt, frac);
    addSample(elapsed, lon, lat, alt);
  }

  // ── CRUISE (great circle at altitude) ────────────────────
  const cruiseSamples = 1000;
  for (let i = 0; i <= cruiseSamples; i++) {
    const frac = i / cruiseSamples;
    const elapsed = t_climbEnd + frac * timing.cruise;

    const gcFrac = lerp(0.0, 0.80, frac); // Cover 0% to 80% of the route
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);

    let alt = CRUISE_ALTITUDE;
    if (frac < 0.1) {
      // Initial climb from climbEndAlt to CRUISE_ALTITUDE
      alt = lerp(climbEndAlt, CRUISE_ALTITUDE, frac / 0.1);
    }

    addSample(elapsed, lon, lat, alt);
  }

  // ── DESCENT (cruise → approach altitude) ─────────────────
  const descentSamples = 100;
  for (let i = 0; i <= descentSamples; i++) {
    const frac = i / descentSamples;
    const elapsed = t_cruiseEnd + frac * timing.descent;

    // Finish the remaining 20% of the great circle route
    const gcFrac = lerp(0.80, 1.0, frac);
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);
    const alt = lerp(CRUISE_ALTITUDE, approachEntryAlt, easeInOutCubic(frac));

    addSample(elapsed, lon, lat, alt);
  }

  // ── LANDING (approach → touchdown, with speed profile) ───
  const landingSamples = 100;
  const approachHorizDistM = haversineDistMeters(
    approachEntryLat, approachEntryLon,
    landingTargetLat, landingTargetLon
  );
  const landingCumTimes = computeCumulativeTimes(
    approachHorizDistM, approachEntryAlt, landingTargetElev, destElev, landingSamples, easeOutQuad
  );

  for (let i = 0; i <= landingSamples; i++) {
    const frac = i / landingSamples;
    const elapsed = t_descentEnd + landingCumTimes[i];
    const lon = lerp(approachEntryLon, landingTargetLon, frac);
    const lat = lerp(approachEntryLat, landingTargetLat, frac);
    const alt = lerp(approachEntryAlt, landingTargetElev, easeOutQuad(frac));
    addSample(elapsed, lon, lat, alt);
  }

  // ── ARRIVAL (rollout → taxi → gate) ──────────────────────
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
    if (elapsed <= t_cruiseEnd) return "cruise";
    if (elapsed <= t_descentEnd) return "descent";
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

  // ── RUNWAY HOLD ────────────────────────────────────────
  const firstRwyTarget = route.runwayWaypoints[0] ?? route.liftoffPoint;
  const rwyBearing = computeBearingRad(route.runwayThreshold, firstRwyTarget);

  for (let i = 0; i <= 3; i++) {
    const frac = i / 3;
    const elapsed = t + frac * timing.runwayHold;
    addSample(
      elapsed,
      route.runwayThreshold.lon + Math.sin(rwyBearing) * DRIFT * (frac * timing.runwayHold),
      route.runwayThreshold.lat + Math.cos(rwyBearing) * DRIFT * (frac * timing.runwayHold),
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
 * Rollout decelerates from 300 → 50 km/h, taxi at constant 50 km/h.
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
