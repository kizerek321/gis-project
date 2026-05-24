/**
 * Airport database — ~30 major world airports with coordinates,
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
  // TO DOs
  NRT: {
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
  JFK: {
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
  SYD: {
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
