/**
 * Flight path engine — generates realistic departure → cruise → landing paths.
 *
 * For airports with a DepartureRoute (e.g. GDN), the plane follows real GPS
 * waypoints: gate hold → taxi → runway hold → takeoff roll → liftoff.
 * For other airports, a generic heading-based departure is used.
 */
import * as Cesium from "cesium";
import type { Airport, DepartureRoute, ArrivalRoute, RouteWaypoint } from "./airports";
import { getDistanceKm, DEPARTURE_ROUTES, getArrivalRoute } from "./airports";

// ─── Types ──────────────────────────────────────────────────

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

// ─── Constants ──────────────────────────────────────────────

const CRUISE_ALTITUDE = 10000;     // meters (~33,000 ft)
const CLIMB_EXIT_ALT = 3000;       // altitude at end of takeoff / start of climb
const APPROACH_ENTRY_ALT = 3000;   // altitude at start of final approach

// Timing for generic departure (airports without a route)
const SIMPLE_TAKEOFF_DURATION = 20;  // seconds

// Timing for route-based departure
const TAXI_SPEED = 6;               // seconds per taxi segment
const TAKEOFF_ROLL_TIMES = [5, 4, 3, 2]; // seconds per runway segment (accelerating)

// Post-departure phases
const CLIMB_DURATION = 45;          // seconds
const DESCENT_DURATION = 45;        // seconds
const LANDING_DURATION = 30;        // seconds
const MIN_CRUISE_DURATION = 15;     // seconds

// Arrival phases (route-based)
const ROLLOUT_SEGMENT_TIME = 4;     // seconds per runway rollout segment
const ARRIVAL_TAXI_SPEED = 6;       // seconds per taxi segment
const ARRIVAL_GATE_HOLD = 5;        // seconds parked at gate
const CRUISE_SCALE = 0.06;          // seconds per km of distance

// ─── Public API ─────────────────────────────────────────────

/**
 * Calculate flight timing plan (used for preview before starting).
 */
export function planFlight(departure: Airport, destination: Airport): FlightPlan {
  const distanceKm = getDistanceKm(departure, destination);
  const route = DEPARTURE_ROUTES[departure.code] ?? null;
  const arrival = getArrivalRoute(destination.code);
  const cruiseDuration = Math.max(MIN_CRUISE_DURATION, distanceKm * CRUISE_SCALE);

  // ── Departure timing ───────────────────────────────────
  let gateTime = 0, taxiTime = 0, runwayHoldTime = 0, takeoffTime: number;

  if (route) {
    const taxiSegments = route.taxiWaypoints.length + 1;
    taxiTime = taxiSegments * TAXI_SPEED;
    const rollSegments = route.runwayWaypoints.length + 1;
    takeoffTime = TAKEOFF_ROLL_TIMES.slice(0, rollSegments)
      .reduce((sum, t) => sum + t, 0);
    gateTime = route.gateHoldTime;
    runwayHoldTime = route.runwayHoldTime;
  } else {
    takeoffTime = SIMPLE_TAKEOFF_DURATION;
  }

  // ── Arrival timing ─────────────────────────────────────
  let rolloutTime = 0, arrivalTaxiTime = 0, arrivalGateTime = 0;

  if (arrival) {
    const rolloutSegments = arrival.rolloutWaypoints.length + 1; // +1 for runwayEnd
    rolloutTime = rolloutSegments * ROLLOUT_SEGMENT_TIME;
    const arrTaxiSegments = arrival.taxiToGateWaypoints.length + 1; // +1 for gate
    arrivalTaxiTime = arrTaxiSegments * ARRIVAL_TAXI_SPEED;
    arrivalGateTime = ARRIVAL_GATE_HOLD;
  }

  const timing: FlightTiming = {
    gate: gateTime,
    taxi: taxiTime,
    runwayHold: runwayHoldTime,
    takeoff: takeoffTime,
    climb: CLIMB_DURATION,
    cruise: cruiseDuration,
    descent: DESCENT_DURATION,
    landing: LANDING_DURATION,
    rollout: rolloutTime,
    arrivalTaxi: arrivalTaxiTime,
    arrivalGate: arrivalGateTime,
  };

  const totalDuration = timing.gate + timing.taxi + timing.runwayHold +
    timing.takeoff + timing.climb + timing.cruise + timing.descent +
    timing.landing + timing.rollout + timing.arrivalTaxi + timing.arrivalGate;

  return { departure, destination, distanceKm, departureRoute: route, arrivalRoute: arrival, timing, totalDuration };
}

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

  // ── Phase time boundaries ──────────────────────────────
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

  // ── Determine liftoff position & great circle ──────────
  let liftoffLat: number;
  let liftoffLon: number;
  let runwayHeadingRad: number;

  if (departureRoute) {
    liftoffLat = departureRoute.liftoffPoint.lat;
    liftoffLon = departureRoute.liftoffPoint.lon;
    const prevPoint = departureRoute.runwayWaypoints[departureRoute.runwayWaypoints.length - 1] ?? departureRoute.runwayThreshold;
    runwayHeadingRad = computeBearingRad(prevPoint, departureRoute.liftoffPoint);
  } else {
    // Generic: a point slightly ahead along runway heading
    runwayHeadingRad = Cesium.Math.toRadians(departure.runwayHeading);
    liftoffLon = departure.lon + Math.sin(runwayHeadingRad) * 0.02;
    liftoffLat = departure.lat + Math.cos(runwayHeadingRad) * 0.02;
  }

  //(0.1 ~ 11 km)
  const climbDistanceDegrees = 0.1; 
  
  const climbExitLon = liftoffLon + Math.sin(runwayHeadingRad) * climbDistanceDegrees;
  const climbExitLat = liftoffLat + Math.cos(runwayHeadingRad) * climbDistanceDegrees;

  // ── Determine landing target ───────────────────────────
  // If we have an arrival route, land at the touchdownPoint;
  // otherwise land at the airport center.
  let landingTargetLat: number;
  let landingTargetLon: number;
  let landingTargetElev: number;

  if (arrivalRoute) {
    landingTargetLat = arrivalRoute.touchdownPoint.lat;
    landingTargetLon = arrivalRoute.touchdownPoint.lon;
    landingTargetElev = destElev;
  } else {
    landingTargetLat = destination.lat;
    landingTargetLon = destination.lon;
    landingTargetElev = destElev;
  }

  // ── Approach entry point ───────────────────────────────
  // Compute approach direction: bearing from touchdown toward the "incoming" side
  const destHeadingRad = arrivalRoute
    ? computeBearingRad(arrivalRoute.rolloutWaypoints[0] ?? arrivalRoute.runwayEnd, arrivalRoute.touchdownPoint)
    : Cesium.Math.toRadians((destination.runwayHeading + 180) % 360);
  
  // Increase approach distance (0.25 degrees ≈ 27 km) for a shallower slope
  const approachDistanceDegrees = 0.25; 
  const approachEntryLon = landingTargetLon + Math.sin(destHeadingRad) * approachDistanceDegrees;
  const approachEntryLat = landingTargetLat + Math.cos(destHeadingRad) * approachDistanceDegrees;

  const climbExitCartographic = Cesium.Cartographic.fromDegrees(climbExitLon, climbExitLat);
  const approachEntryCartographic = Cesium.Cartographic.fromDegrees(approachEntryLon, approachEntryLat);
  
  // Great circle connects the climb exit to the approach entry point
  const geodesic = new Cesium.EllipsoidGeodesic(climbExitCartographic, approachEntryCartographic);

  // ═══════════════════════════════════════════════════════
  // SAMPLE GENERATION
  // ═══════════════════════════════════════════════════════

  function addSample(elapsed: number, lon: number, lat: number, alt: number): void {
    const time = Cesium.JulianDate.addSeconds(start, elapsed, new Cesium.JulianDate());
    position.addSample(time, Cesium.Cartesian3.fromDegrees(lon, lat, alt));
  }

  if (departureRoute) {
    // ─── ROUTE-BASED DEPARTURE ─────────────────────────
    buildRouteDeparture(departureRoute, elev, addSample, timing);
  } else {
    // ─── SIMPLE DEPARTURE ──────────────────────────────
    buildSimpleDeparture(departure, addSample, timing);
  }

  // ─── CLIMB (liftoff → cruise altitude) ────────────────
  const climbSamples = 100;
  for (let i = 0; i <= climbSamples; i++) {
    const frac = i / climbSamples;
    const elapsed = t_takeoffEnd + frac * timing.climb;

    //linear
    const lon = lerp(liftoffLon, climbExitLon, frac);
    const lat = lerp(liftoffLat, climbExitLat, frac);
    const alt = lerp(elev + 50, CLIMB_EXIT_ALT, frac);
    
    addSample(elapsed, lon, lat, alt);
  }

  // ─── CRUISE (great circle at altitude) ────────────────
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
        alt = lerp(CLIMB_EXIT_ALT, CRUISE_ALTITUDE, frac / 0.1);
    }
    
    addSample(elapsed, lon, lat, alt);
  }

  // ─── DESCENT (cruise → approach altitude) ─────────────
  const descentSamples = 100;
  for (let i = 0; i <= descentSamples; i++) {
    const frac = i / descentSamples;
    const elapsed = t_cruiseEnd + frac * timing.descent;
    
    // Finish the remaining 20% of the great circle route
    const gcFrac = lerp(0.80, 1.0, frac);
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);
    const alt = lerp(CRUISE_ALTITUDE, APPROACH_ENTRY_ALT, easeInOutCubic(frac));
    
    addSample(elapsed, lon, lat, alt);
  }

  // ─── LANDING (approach → touchdown) ───────────────────
  const landingSamples = 100;
  for (let i = 0; i <= landingSamples; i++) {
    const frac = i / landingSamples;
    const elapsed = t_descentEnd + frac * timing.landing;
    
    const lon = lerp(approachEntryLon, landingTargetLon, frac);
    const lat = lerp(approachEntryLat, landingTargetLat, frac);
    const alt = lerp(APPROACH_ENTRY_ALT, landingTargetElev, easeOutQuad(frac));
    
    addSample(elapsed, lon, lat, alt);
  }

  // ─── ARRIVAL (rollout → taxi → gate) ──────────────────
  if (arrivalRoute) {
    buildRouteArrival(arrivalRoute, destElev, addSample, timing, t_landingEnd);
  }

  // ═══════════════════════════════════════════════════════
  // ENTITY
  // ═══════════════════════════════════════════════════════

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

  // ═══════════════════════════════════════════════════════
  // PHASE RESOLVER
  // ═══════════════════════════════════════════════════════

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

// ═══════════════════════════════════════════════════════════
// DEPARTURE BUILDERS
// ═══════════════════════════════════════════════════════════

type AddSampleFn = (elapsed: number, lon: number, lat: number, alt: number) => void;

/**
 * Build samples for a route-based departure: gate → taxi → hold → takeoff roll → liftoff.
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

  // ── TAXI ───────────────────────────────────────────────
  // Build full taxi path: gate → wp1 → wp2 → ... → threshold
  const taxiPath: RouteWaypoint[] = [
    route.gate,
    ...route.taxiWaypoints,
    route.runwayThreshold,
  ];

  const numTaxiSegments = taxiPath.length - 1;
  const segDuration = timing.taxi / numTaxiSegments;

  for (let seg = 0; seg < numTaxiSegments; seg++) {
    const from = taxiPath[seg];
    const to = taxiPath[seg + 1];
    // 3 sub-samples per segment (start, mid, end) — skip start for seg > 0 to avoid duplicates
    const subStart = seg === 0 ? 0 : 1;
    for (let sub = subStart; sub <= 3; sub++) {
      const frac = sub / 3;
      const elapsed = t + seg * segDuration + frac * segDuration;
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);
      addSample(elapsed, lon, lat, elevation);
    }
  }
  t += timing.taxi;

  // ── RUNWAY HOLD ────────────────────────────────────────
  // Plane stops at threshold, pointing down the runway
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

  // ── TAKEOFF ROLL ───────────────────────────────────────
  // Build runway path: threshold → rwyWp1 → rwyWp2 → ... → liftoff
  const rwyPath: RouteWaypoint[] = [
    route.runwayThreshold,
    ...route.runwayWaypoints,
    route.liftoffPoint,
  ];

  const numRwySegments = rwyPath.length - 1;
  // Use TAKEOFF_ROLL_TIMES for acceleration (or distribute evenly if fewer segments)
  const rollTimes: number[] = [];
  for (let i = 0; i < numRwySegments; i++) {
    rollTimes.push(TAKEOFF_ROLL_TIMES[i] ?? TAKEOFF_ROLL_TIMES[TAKEOFF_ROLL_TIMES.length - 1]);
  }
  const totalRollTime = rollTimes.reduce((s, t) => s + t, 0);
  // Scale to fit actual timing.takeoff
  const rollScale = timing.takeoff / totalRollTime;

  let rwyT = t;
  for (let seg = 0; seg < numRwySegments; seg++) {
    const from = rwyPath[seg];
    const to = rwyPath[seg + 1];
    const segTime = rollTimes[seg] * rollScale;
    const isLastSegment = seg === numRwySegments - 1;

    // 4 sub-samples per segment for smooth acceleration
    const subStart = seg === 0 ? 0 : 1;
    for (let sub = subStart; sub <= 4; sub++) {
      const frac = sub / 4;
      const elapsed = rwyT + frac * segTime;
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);

      // Start climbing in the last segment (liftoff)
      let alt = elevation;
      if (isLastSegment && frac > 0.5) {
        const liftFrac = (frac - 0.5) / 0.5;
        //alt = elevation + easeInQuad(liftFrac) * CLIMB_EXIT_ALT;
        alt = elevation + easeInQuad(liftFrac) * 50;
      }

      addSample(elapsed, lon, lat, alt);
    }
    rwyT += segTime;
  }
}

/**
 * Build samples for a generic heading-based departure.
 */
function buildSimpleDeparture(
  departure: Airport,
  addSample: AddSampleFn,
  timing: FlightTiming
): void {
  const hdgRad = Cesium.Math.toRadians(departure.runwayHeading);
  const samples = 20;

  for (let i = 0; i <= samples; i++) {
    const frac = i / samples;
    const elapsed = frac * timing.takeoff;
    const dist = frac * 0.02; // degrees along heading
    const lon = departure.lon + Math.sin(hdgRad) * dist;
    const lat = departure.lat + Math.cos(hdgRad) * dist;

    let alt: number;
    if (frac < 0.4) {
      alt = departure.elevation;
    } else {
      const liftFrac = (frac - 0.4) / 0.6;
      alt = departure.elevation + easeInQuad(liftFrac) * CLIMB_EXIT_ALT;
    }

    addSample(elapsed, lon, lat, alt);
  }
}

// ═══════════════════════════════════════════════════════════
// ARRIVAL BUILDER
// ═══════════════════════════════════════════════════════════

/**
 * Build samples for a route-based arrival: rollout → taxi to gate → gate hold.
 * Called after the landing phase ends (plane has touched down at touchdownPoint).
 */
function buildRouteArrival(
  route: ArrivalRoute,
  elevation: number,
  addSample: AddSampleFn,
  timing: FlightTiming,
  tStart: number
): void {
  let t = tStart;

  // ── ROLLOUT (touchdown → rolloutWaypoints → runwayEnd) ──
  const rolloutPath: RouteWaypoint[] = [
    route.touchdownPoint,
    ...route.rolloutWaypoints,
    route.runwayEnd,
  ];

  const numRolloutSegments = rolloutPath.length - 1;
  const rolloutSegDuration = timing.rollout / numRolloutSegments;

  for (let seg = 0; seg < numRolloutSegments; seg++) {
    const from = rolloutPath[seg];
    const to = rolloutPath[seg + 1];
    const subStart = seg === 0 ? 0 : 1;
    for (let sub = subStart; sub <= 4; sub++) {
      const frac = sub / 4;
      const elapsed = t + seg * rolloutSegDuration + frac * rolloutSegDuration;
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);
      addSample(elapsed, lon, lat, elevation);
    }
  }
  t += timing.rollout;

  // ── ARRIVAL TAXI (runwayEnd → taxiToGateWaypoints → gate) ──
  const taxiPath: RouteWaypoint[] = [
    route.runwayEnd,
    ...route.taxiToGateWaypoints,
    route.gate,
  ];

  const numTaxiSegments = taxiPath.length - 1;
  const taxiSegDuration = timing.arrivalTaxi / numTaxiSegments;

  for (let seg = 0; seg < numTaxiSegments; seg++) {
    const from = taxiPath[seg];
    const to = taxiPath[seg + 1];
    const subStart = seg === 0 ? 0 : 1;
    for (let sub = subStart; sub <= 3; sub++) {
      const frac = sub / 3;
      const elapsed = t + seg * taxiSegDuration + frac * taxiSegDuration;
      const lon = lerp(from.lon, to.lon, frac);
      const lat = lerp(from.lat, to.lat, frac);
      addSample(elapsed, lon, lat, elevation);
    }
  }
  t += timing.arrivalTaxi;

  // ── ARRIVAL GATE HOLD ─────────────────────────────────
  // Tiny drift so VelocityOrientationProperty can compute heading
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

// ─── Math helpers ───────────────────────────────────────────

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
