/**
 * Flight path engine — generates realistic departure → cruise → landing paths.
 *
 * For airports with a DepartureRoute (e.g. GDN), the plane follows real GPS
 * waypoints: gate hold → taxi → runway hold → takeoff roll → liftoff.
 * For other airports, a generic heading-based departure is used.
 */
import * as Cesium from "cesium";
import type { Airport, DepartureRoute, RouteWaypoint } from "./airports";
import { getDistanceKm, DEPARTURE_ROUTES } from "./airports";

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
  | "arrived";      // Parked at destination

export interface FlightTiming {
  gate: number;         // 0 if no departure route
  taxi: number;         // 0 if no departure route
  runwayHold: number;   // 0 if no departure route
  takeoff: number;      // runway roll (route) or generic takeoff (simple)
  climb: number;
  cruise: number;
  descent: number;
  landing: number;
}

export interface FlightPlan {
  departure: Airport;
  destination: Airport;
  distanceKm: number;
  departureRoute: DepartureRoute | null;
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
const CRUISE_SCALE = 0.06;          // seconds per km of distance

// ─── Public API ─────────────────────────────────────────────

/**
 * Calculate flight timing plan (used for preview before starting).
 */
export function planFlight(departure: Airport, destination: Airport): FlightPlan {
  const distanceKm = getDistanceKm(departure, destination);
  const route = DEPARTURE_ROUTES[departure.code] ?? null;
  const cruiseDuration = Math.max(MIN_CRUISE_DURATION, distanceKm * CRUISE_SCALE);

  let timing: FlightTiming;

  if (route) {
    // Route-based departure: gate + taxi + hold + roll
    const taxiSegments = route.taxiWaypoints.length + 1; // +1 for gate→first wp
    const taxiTime = taxiSegments * TAXI_SPEED;
    const rollSegments = route.runwayWaypoints.length + 1; // +1 for liftoff
    const rollTime = TAKEOFF_ROLL_TIMES.slice(0, rollSegments)
      .reduce((sum, t) => sum + t, 0);

    timing = {
      gate: route.gateHoldTime,
      taxi: taxiTime,
      runwayHold: route.runwayHoldTime,
      takeoff: rollTime,
      climb: CLIMB_DURATION,
      cruise: cruiseDuration,
      descent: DESCENT_DURATION,
      landing: LANDING_DURATION,
    };
  } else {
    // Simple departure
    timing = {
      gate: 0,
      taxi: 0,
      runwayHold: 0,
      takeoff: SIMPLE_TAKEOFF_DURATION,
      climb: CLIMB_DURATION,
      cruise: cruiseDuration,
      descent: DESCENT_DURATION,
      landing: LANDING_DURATION,
    };
  }

  const totalDuration = timing.gate + timing.taxi + timing.runwayHold +
    timing.takeoff + timing.climb + timing.cruise + timing.descent + timing.landing;

  return { departure, destination, distanceKm, departureRoute: route, timing, totalDuration };
}

/**
 * Build the complete flight entity with sampled positions.
 */
export function createFlight(
  viewer: Cesium.Viewer,
  plan: FlightPlan
): FlightResult {
  const { departure, destination, timing, totalDuration, departureRoute } = plan;

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
    interpolationDegree: 3,
    interpolationAlgorithm: Cesium.HermitePolynomialApproximation,
  });

  const elev = departure.elevation;

  // ── Phase time boundaries ──────────────────────────────
  const t_gateEnd = timing.gate;
  const t_taxiEnd = t_gateEnd + timing.taxi;
  const t_holdEnd = t_taxiEnd + timing.runwayHold;
  const t_takeoffEnd = t_holdEnd + timing.takeoff;
  const t_climbEnd = t_takeoffEnd + timing.climb;
  const t_cruiseEnd = t_climbEnd + timing.cruise;
  const t_descentEnd = t_cruiseEnd + timing.descent;
  // t_landingEnd = totalDuration

  // ── Determine liftoff position & great circle ──────────
  let liftoffLat: number;
  let liftoffLon: number;

  if (departureRoute) {
    liftoffLat = departureRoute.liftoffPoint.lat;
    liftoffLon = departureRoute.liftoffPoint.lon;
  } else {
    // Generic: a point slightly ahead along runway heading
    const hdgRad = Cesium.Math.toRadians(departure.runwayHeading);
    liftoffLon = departure.lon + Math.sin(hdgRad) * 0.02;
    liftoffLat = departure.lat + Math.cos(hdgRad) * 0.02;
  }

  const liftoffCartographic = Cesium.Cartographic.fromDegrees(liftoffLon, liftoffLat);
  const destCartographic = Cesium.Cartographic.fromDegrees(destination.lon, destination.lat);
  const geodesic = new Cesium.EllipsoidGeodesic(liftoffCartographic, destCartographic);

  const destHeadingRad = Cesium.Math.toRadians((destination.runwayHeading + 180) % 360);

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
  const climbSamples = 20;
  for (let i = 0; i <= climbSamples; i++) {
    const frac = i / climbSamples;
    const elapsed = t_takeoffEnd + frac * timing.climb;
    // Move along first 10% of the great circle during climb
    const gcFrac = easeInOutCubic(frac) * 0.10;
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);
    const alt = lerp(CLIMB_EXIT_ALT, CRUISE_ALTITUDE, easeInOutCubic(frac));
    addSample(elapsed, lon, lat, alt);
  }

  // ─── CRUISE (great circle at altitude) ────────────────
  const cruiseSamples = 40;
  for (let i = 0; i <= cruiseSamples; i++) {
    const frac = i / cruiseSamples;
    const elapsed = t_climbEnd + frac * timing.cruise;
    const gcFrac = lerp(0.10, 0.90, frac);
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);
    // Subtle altitude variation for realism
    const alt = CRUISE_ALTITUDE + Math.sin(frac * Math.PI * 4) * 200;
    addSample(elapsed, lon, lat, alt);
  }

  // ─── DESCENT (cruise → approach altitude) ─────────────
  const descentSamples = 20;
  for (let i = 0; i <= descentSamples; i++) {
    const frac = i / descentSamples;
    const elapsed = t_cruiseEnd + frac * timing.descent;
    const gcFrac = lerp(0.90, 1.0, easeInOutCubic(frac));
    const interp = geodesic.interpolateUsingFraction(gcFrac);
    const lon = Cesium.Math.toDegrees(interp.longitude);
    const lat = Cesium.Math.toDegrees(interp.latitude);
    const alt = lerp(CRUISE_ALTITUDE, APPROACH_ENTRY_ALT, easeInOutCubic(frac));
    addSample(elapsed, lon, lat, alt);
  }

  // ─── LANDING (approach → touchdown) ───────────────────
  const landingSamples = 20;
  for (let i = 0; i <= landingSamples; i++) {
    const frac = i / landingSamples;
    const elapsed = t_descentEnd + frac * timing.landing;
    // Approach aligned with destination runway
    const approachDist = (1 - frac) * 0.02;
    const lon = destination.lon + Math.sin(destHeadingRad) * approachDist;
    const lat = destination.lat + Math.cos(destHeadingRad) * approachDist;
    let alt: number;
    if (frac < 0.7) {
      alt = lerp(APPROACH_ENTRY_ALT, destination.elevation, easeOutQuad(frac / 0.7));
    } else {
      alt = destination.elevation;
    }
    addSample(elapsed, lon, lat, alt);
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
    if (elapsed <= totalDuration) return "landing";
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
        alt = elevation + easeInQuad(liftFrac) * CLIMB_EXIT_ALT;
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
