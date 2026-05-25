/**
 * Airport database — 5 major world airports with coordinates,
 * elevations, and primary runway headings for realistic takeoff/landing.
 */

export interface Airport {
  code: string;
  city: string;
  country: string;
  lon: number;
  lat: number;
  elevation: number;      // meters above sea level
  runwayHeading: number;  // degrees true — takeoff/landing direction
}

export const AIRPORTS: Airport[] = [
  { code: "GDN", city: "Gdańsk",       country: "Poland",        lon: 18.4526,  lat: 54.3809, elevation: 165,   runwayHeading: 112 },
  { code: "WAW", city: "Warsaw",       country: "Poland",        lon: 20.9671,  lat: 52.1657, elevation: 132,  runwayHeading: 110 },
  { code: "NRT", city: "Tokyo Narita", country: "Japan",         lon: 140.3929, lat: 35.7720, elevation: 80,   runwayHeading: 160 },
  { code: "JFK", city: "New York",     country: "USA",           lon: -73.7781, lat: 40.6413, elevation: 0,    runwayHeading: 310 },
  { code: "SYD", city: "Sydney",       country: "Australia",     lon: 151.1772, lat: -33.9461,elevation: 30,    runwayHeading: 160 },
];

/**
 * Haversine distance between two airports in kilometers.
 */
export function getDistanceKm(a: Airport, b: Airport): number {
  const R = 6371;
  const dLat = toRad(b.lat - a.lat);
  const dLon = toRad(b.lon - a.lon);
  const sinHalfDLat = Math.sin(dLat / 2);
  const sinHalfDLon = Math.sin(dLon / 2);
  const h = sinHalfDLat * sinHalfDLat +
    Math.cos(toRad(a.lat)) * Math.cos(toRad(b.lat)) * sinHalfDLon * sinHalfDLon;
  return R * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}

function toRad(deg: number): number {
  return deg * Math.PI / 180;
}

/**
 * Find airport by IATA code.
 */
export function findAirport(code: string): Airport | undefined {
  return AIRPORTS.find(a => a.code === code);
}

// ─── Departure Route Waypoints ────────────────────────────────

export interface RouteWaypoint {
  lat: number;
  lon: number;
}

export interface DepartureRoute {
  /** Plane parks here at the gate / stand */
  gate: RouteWaypoint;
  /** Seconds to wait at gate before moving */
  gateHoldTime: number;
  /** Taxi waypoints from gate to runway threshold */
  taxiWaypoints: RouteWaypoint[];
  /** Start of the runway */
  runwayThreshold: RouteWaypoint;
  /** Seconds to hold at threshold ("cleared for takeoff") */
  runwayHoldTime: number;
  /** Intermediate points along the runway during takeoff roll */
  runwayWaypoints: RouteWaypoint[];
  /** Where the plane lifts off at the end of the runway */
  liftoffPoint: RouteWaypoint;
}

/**
 * Airport-specific departure routes with real GPS waypoints.
 * Airports not in this map will use a generic heading-based departure.
 */
export const DEPARTURE_ROUTES: Record<string, DepartureRoute> = {
  GDN: {
    gate:             { lat: 54.382008, lon: 18.461487 },
    gateHoldTime:     10,
    taxiWaypoints: [
      { lat: 54.380842, lon: 18.460441 },
      { lat: 54.381409, lon: 18.457662 },
      { lat: 54.381367, lon: 18.456940 },
      { lat: 54.381115, lon: 18.456327 },
    ],
    runwayThreshold:  { lat: 54.380190, lon: 18.455677 },
    runwayHoldTime:   3,
    runwayWaypoints: [
      { lat: 54.379770, lon: 18.457211 },
      { lat: 54.377248, lon: 18.467441 },
      { lat: 54.375650, lon: 18.474154 },
    ],
    liftoffPoint:     { lat: 54.372255, lon: 18.487398 },
  },
  WAW: {
    gate:             { lat: 52.171892, lon: 20.969919 },
    gateHoldTime:     10,
    taxiWaypoints: [ 
      { lat: 52.171098, lon: 20.966428 },
      { lat: 52.170105, lon: 20.963315 },
    ],
    runwayThreshold:  { lat: 52.170061, lon: 20.963394 },
    runwayHoldTime:   3,
    runwayWaypoints: [
      { lat: 52.165569, lon: 20.967297 },
      { lat: 52.154933, lon: 20.976548 }
    ],
    liftoffPoint:     { lat: 52.149808, lon: 20.980909 },
  },
  NRT: { 
    gate:             { lat: 35.760170, lon: 140.385191 },
    gateHoldTime:     10,
    taxiWaypoints: [
      { lat: 35.760439, lon: 140.383330 },
      { lat: 35.760223, lon: 140.382045 },
      { lat: 35.759217, lon: 140.381007 },
      { lat: 35.758333, lon: 140.381124 },
      { lat: 35.757096, lon: 140.381465 }
    ],
    runwayThreshold:  { lat: 35.756455, lon: 140.381272 },
    runwayHoldTime:   3,
    runwayWaypoints: [
      { lat: 35.755394, lon: 140.381974 },
      { lat: 35.753990, lon: 140.382993 },
      { lat: 35.751962, lon: 140.384492 },
    ],
    liftoffPoint:     { lat: 35.750129, lon: 140.385833 },
  },
  JFK: {
    gate:             { lat: 40.636710, lon: -73.780462 },
    gateHoldTime:     10,
    taxiWaypoints: [
      { lat: 40.635878, lon: -73.778524 },
      { lat: 40.635053, lon: -73.777106 },
      { lat: 40.633375, lon: -73.778387 },
      { lat: 40.632725, lon: -73.777323 },
      { lat: 40.631554, lon: -73.774769 },
      { lat: 40.629418, lon: -73.770006 },
      { lat: 40.629356, lon: -73.768707 },
      { lat: 40.630329, lon: -73.766623 }
    ],
    runwayThreshold:  { lat: 40.632042, lon: -73.765196 },
    runwayHoldTime:   3,
    runwayWaypoints: [
      { lat: 40.630487, lon: -73.766430 },
      { lat: 40.627865, lon: -73.768505 },
      { lat: 40.626025, lon: -73.769896 }
    ],
    liftoffPoint:     { lat: 40.624592, lon: -73.771044 },
  },
  SYD: {
    gate:             { lat: -33.936766, lon: 151.178867 },
    gateHoldTime:     10,
    taxiWaypoints: [
      { lat: -33.936701, lon: 151.177335 },
      { lat: -33.936574, lon: 151.175487 }
    ],
    runwayThreshold:  { lat: -33.936878, lon: 151.175557 },
    runwayHoldTime:   3,
    runwayWaypoints: [
      { lat: -33.938854, lon: 151.176035 },
      { lat: -33.940734, lon: 151.176550 },
      { lat: -33.943511, lon: 151.177400 },
      { lat: -33.945256, lon: 151.177943 },
      { lat: -33.947001, lon: 151.178403 },
      { lat: -33.951170, lon: 151.179447 },
    ],
    liftoffPoint:     { lat: -33.951981, lon: 151.179671 },
  },
};

/**
 * Compute bearing (heading) in radians from point A to point B.
 * Returns 0 = North, π/2 = East, π = South, 3π/2 = West.
 */
export function computeBearing(from: RouteWaypoint, to: RouteWaypoint): number {
  const dLon = toRad(to.lon - from.lon);
  const lat1 = toRad(from.lat);
  const lat2 = toRad(to.lat);
  const y = Math.sin(dLon) * Math.cos(lat2);
  const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
  return (Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI);
}
