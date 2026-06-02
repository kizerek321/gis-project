# ✈️ Flight Simulator — CesiumJS

A real-time 3D flight simulator built on **CesiumJS** and **Vite + TypeScript**. Select a departure and destination airport, press "Start Flight", and watch a physically realistic airline flight unfold on a photorealistic 3D globe — from gate departure to arrival gate — with accurate kinematics, geodesic navigation, and atmospheric rendering.

---

## Table of Contents

- [Overview](#overview)
- [Technology Stack](#technology-stack)
- [Project Structure](#project-structure)
- [Physics & Simulation Model](#physics--simulation-model)
  - [1. Spherical Earth Geometry (Geodesics)](#1-spherical-earth-geometry-geodesics)
  - [2. Haversine Distance Formula](#2-haversine-distance-formula)
  - [3. Great-Circle Navigation](#3-great-circle-navigation)
  - [4. Forward Geodesic Projection](#4-forward-geodesic-projection)
  - [5. Initial Bearing (Azimuth) Computation](#5-initial-bearing-azimuth-computation)
  - [6. Altitude-Dependent Speed Profile](#6-altitude-dependent-speed-profile)
  - [7. Constant-Acceleration Kinematics (Takeoff Roll)](#7-constant-acceleration-kinematics-takeoff-roll)
  - [8. Linear Deceleration Kinematics (Rollout)](#8-linear-deceleration-kinematics-rollout)
  - [9. Variable-Speed Numerical Integration (Climb & Approach)](#9-variable-speed-numerical-integration-climb--approach)
  - [10. Analytical Time Integration for Linear Speed Ramp](#10-analytical-time-integration-for-linear-speed-ramp)
  - [11. Rate-Limited Turn Simulation (Standard Rate Turn)](#11-rate-limited-turn-simulation-standard-rate-turn)
  - [12. Easing Functions (Non-Linear Motion Curves)](#12-easing-functions-non-linear-motion-curves)
  - [13. 3D Orientation — Velocity-Derived Quaternion](#13-3d-orientation--velocity-derived-quaternion)
  - [14. Pitch (Flare) Dynamics](#14-pitch-flare-dynamics)
  - [15. Bank Angle (Roll) from Turn Rate](#15-bank-angle-roll-from-turn-rate)
  - [16. Hermite Polynomial Interpolation](#16-hermite-polynomial-interpolation)
  - [17. Terrain & Elevation Model](#17-terrain--elevation-model)
  - [18. Atmospheric & Visual Rendering](#18-atmospheric--visual-rendering)
- [Flight Phases](#flight-phases)
- [Airport Database](#airport-database)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [License](#license)

---

## Overview

This simulator models a complete commercial airline flight with **11 distinct phases**, from gate pushback to arrival gate parking. Every aspect of the flight — speed, altitude, heading, bank angle, pitch — is computed from physical models rather than canned animations, producing realistic behavior across any route between the five supported airports.

The simulation runs on a full WGS-84 ellipsoidal globe with satellite imagery and Cesium World Terrain, giving the user a photorealistic first-person or third-person view of the entire flight.

---

## Technology Stack

| Layer           | Technology                                                              |
| --------------- | ----------------------------------------------------------------------- |
| 3D Globe & Map  | **CesiumJS** v1.141+ (WebGL, WGS-84 ellipsoid, world terrain)          |
| Language        | **TypeScript** ~6.0                                                     |
| Build Tool      | **Vite** v8 with `vite-plugin-cesium`                                   |
| 3D Aircraft     | `plane.glb` — glTF 2.0 model rendered via Cesium's `ModelGraphics`      |
| Fonts           | **Inter** (Google Fonts)                                                |
| IDE             | Visual Studio Code with ESLint & Prettier                               |

---

## Project Structure

```
symulator-cesium/
├── index.html             # Application shell — UI panel, Cesium container
├── package.json           # Dependencies: cesium, vite, typescript
├── vite.config.ts         # Vite + Cesium plugin configuration
├── tsconfig.json          # TypeScript compiler options
├── .env                   # VITE_CESIUM_TOKEN (Cesium Ion access token)
├── public/
│   ├── plane.glb          # 3D aircraft model (glTF binary)
│   ├── favicon.svg        # Browser favicon
│   └── icons.svg          # UI icon sprites
└── src/
    ├── main.ts            # Application entry — viewer init, UI events, flight lifecycle
    ├── scene.ts           # Cesium Viewer setup — terrain, atmosphere, post-processing
    ├── flight.ts          # Core flight engine — physics, path generation, orientation
    ├── airports.ts        # Airport database — coordinates, routes, waypoints
    ├── style.css          # UI styling — glassmorphism panel, animations
    └── assets/            # Additional static assets
```

---

## Physics & Simulation Model

Below is a detailed description of every physics concept and mathematical model used in the simulator.

---

### 1. Spherical Earth Geometry (Geodesics)

The entire simulation operates on a **spherical Earth model** with radius **R = 6,371,000 m** (mean Earth radius). All distances between geographic coordinates (latitude/longitude) are computed as great-circle arcs on this sphere rather than flat Euclidean distances. This is essential for correctness — at intercontinental scales (e.g., Gdańsk → Tokyo ≈ 8,600 km), flat-Earth approximations would produce errors of hundreds of kilometers.

The Earth's surface is parameterized in **geodetic coordinates** (latitude φ, longitude λ) measured in degrees and converted to radians for trigonometric calculations.

---

### 2. Haversine Distance Formula

Distance between two geographic points is computed using the **Haversine formula**, which is numerically stable even for small distances (unlike the simpler spherical law of cosines):

```
a = sin²(Δφ/2) + cos(φ₁) · cos(φ₂) · sin²(Δλ/2)
d = R · 2 · atan2(√a, √(1 − a))
```

Where:
- **φ₁, φ₂** — latitudes of the two points (radians)
- **Δφ** — difference in latitude
- **Δλ** — difference in longitude
- **R** — Earth's radius (6,371 km for airport-to-airport, 6,371,000 m for waypoint-to-waypoint)
- **d** — great-circle distance

This formula is used in three contexts:
- `getDistanceKm()` — total airport-to-airport distance for the UI info panel
- `waypointDistMeters()` — segment distances for taxi, runway, and rollout paths
- `haversineDistMeters()` — general-purpose distance between any two lat/lon pairs

---

### 3. Great-Circle Navigation

During the **cruise** and **descent** phases, the aircraft follows a **great-circle path** (the shortest path on a sphere between two points). CesiumJS provides the `EllipsoidGeodesic` class which solves the direct and inverse geodesic problems on the WGS-84 ellipsoid.

The cruise path is built by:
1. Constructing an `EllipsoidGeodesic` from the climb exit point to the approach entry point.
2. Interpolating fractionally along this geodesic to generate 1,000+ position samples.
3. The cruise phase covers **0–80%** of the geodesic, and the descent covers **80–100%**.

This means the plane actually follows the geometrically shortest path across the globe, curving naturally over the sphere — visible on long flights like GDN → NRT where the great circle passes near the North Pole.

---

### 4. Forward Geodesic Projection

To calculate where a point ends up after moving a given distance along a bearing (heading) on the sphere, the simulator uses the **forward geodesic problem** (also known as the "direct problem"):

```
φ₂ = asin(sin(φ₁)·cos(d/R) + cos(φ₁)·sin(d/R)·cos(θ))
λ₂ = λ₁ + atan2(sin(θ)·sin(d/R)·cos(φ₁), cos(d/R) − sin(φ₁)·sin(φ₂))
```

Where:
- **(φ₁, λ₁)** — starting latitude/longitude
- **θ** — bearing (heading) in radians
- **d** — distance to travel (meters)
- **(φ₂, λ₂)** — resulting latitude/longitude

This is implemented in two functions:
- `offsetByBearingMeters()` — used for computing the climb exit point (100 km ahead of liftoff along the runway heading)
- `moveWithHeading()` — used in the step-by-step climb and approach simulations to advance the aircraft's position each time step

---

### 5. Initial Bearing (Azimuth) Computation

The **initial bearing** (forward azimuth) from point A to point B on a sphere:

```
θ = atan2(sin(Δλ)·cos(φ₂), cos(φ₁)·sin(φ₂) − sin(φ₁)·cos(φ₂)·cos(Δλ))
```

Normalized to [0, 2π). This gives the compass heading (North = 0, East = π/2, South = π, West = 3π/2).

Used extensively for:
- Runway heading detection from waypoint sequences
- Target heading computation during rate-limited turns
- Gate-to-taxi bearing for velocity orientation at the gate
- Bank angle computation (comparing headings over time)

---

### 6. Altitude-Dependent Speed Profile

Aircraft speed is modeled as a **piecewise-linear function of altitude above ground level (AGL)**. This simulates the real-world behavior where aircraft accelerate gradually as they climb and decelerate as they descend:

| Altitude Range (AGL) | Speed          | Description                              |
| -------------------- | -------------- | ---------------------------------------- |
| Ground (AGL ≤ 0)     | 50 km/h        | Taxi and ground operations               |
| 0 – 400 m            | 300 km/h       | Post-liftoff / final approach speed      |
| 400 – 800 m          | 300 → 450 km/h | Linear interpolation (early climb)       |
| 800 – 3,000 m        | 450 → 900 km/h | Linear interpolation (main acceleration) |
| Above 3,000 m        | 900 km/h       | Cruise speed (constant)                  |

The speed function `getSpeedForAltitude(altitude, groundElev)` computes AGL as `altitude − groundElev` and applies **linear interpolation** (`lerp`) within each altitude band. The airport's ground elevation is subtracted to correctly compute height above the local terrain, which differs between airports (e.g., WAW at 132 m MSL vs. JFK at 0 m MSL).

---

### 7. Constant-Acceleration Kinematics (Takeoff Roll)

The **takeoff roll** (runway acceleration from rest to liftoff speed) uses the **constant-acceleration kinematic model**:

```
v(t) = a · t           (velocity increases linearly)
x(t) = ½ · a · t²      (position grows quadratically)
```

Derived relationships:
- **Total time**: `T = 2 · D / v_final` (from `v_final = a·T` and `D = ½·a·T²`)
- **Time at position fraction**: `t/T = √(x/D)` (square root gives realistic slow start, fast end)

The aircraft accelerates from **0 to 300 km/h (83.3 m/s)** over the real GPS-measured runway length. Position samples are spaced using `√(distanceFraction)` to produce non-uniform time spacing that matches the quadratic position curve.

During the final 50% of the last runway segment, the aircraft begins a gentle vertical liftoff using an `easeInQuad` curve, rising 50 meters to simulate the rotation and initial climb-out.

---

### 8. Linear Deceleration Kinematics (Rollout)

After touchdown, the **rollout** phase decelerates the aircraft from **300 km/h → 50 km/h** using an **analytical solution for linear speed change**:

```
v(x) = v₀ + (v₁ − v₀) · (x / D)       (speed varies linearly with distance)
T = D · ln(v₁/v₀) / (v₁ − v₀)          (total time — integral of 1/v(x) dx)
timeFrac(f) = ln(v(f)/v₀) / ln(v₁/v₀)  (time fraction at distance fraction f)
```

This produces a physically correct non-linear time progression: the aircraft covers early segments quickly (still fast) and later segments slowly (after braking). The function `linearSpeedTimeFrac()` maps distance fractions to time fractions, and `linearSpeedTotalTime()` gives the total duration.

---

### 9. Variable-Speed Numerical Integration (Climb & Approach)

For the **climb** and **approach** phases, the speed changes continuously with altitude. Since there is no closed-form solution for the arbitrary piecewise speed profile, the duration is computed via **numerical integration** (midpoint rectangle method):

```
T = Σᵢ (Δsᵢ / v(altᵢ_mid))
```

Where each segment `i`:
1. Computes the altitude at segment boundaries using the easing function: `alt = lerp(startAlt, endAlt, ease(f))`
2. Calculates the actual 3D segment distance using the **Pythagorean theorem**: `Δs = √(Δx_horiz² + Δalt²)` — this accounts for the climb angle adding to the total path length
3. Evaluates speed at the midpoint altitude using `getSpeedForAltitude()`

The function `computeVariableSpeedDuration()` uses **100 integration segments** by default, providing sub-second accuracy for the estimated phase duration.

---

### 10. Analytical Time Integration for Linear Speed Ramp

For cases where the speed changes linearly between two values `v₀` and `v₁` over a distance `D`, the simulator uses the **exact analytical integral** rather than numerical approximation:

```
Total time:     T = D · ln(v₁/v₀) / (v₁ − v₀)
Time fraction:  f_t = ln(v(f_d) / v₀) / ln(v₁ / v₀)
```

These are derived from integrating `dt = dx / v(x)` where `v(x) = v₀ + (v₁ − v₀)(x/D)`:

```
T = ∫₀ᴰ dx / v(x) = D · [ln(v₁) − ln(v₀)] / (v₁ − v₀)
```

When `|v₁ − v₀| < 0.01` (approximately constant speed), the formulas degenerate gracefully to `T = D / v₀` to avoid division by zero or logarithm of 1.

---

### 11. Rate-Limited Turn Simulation (Standard Rate Turn)

Both the **climb** and **approach** phases use a **step-by-step kinematic simulation** with rate-limited heading changes. This models the **Standard Rate Turn** used in instrument flying:

- **Maximum turn rate**: 3°/s (0.0524 rad/s) — the standard rate for commercial aircraft
- **Simulation time step**: Δt = 0.5 s
- **Maximum heading change per step**: 3° × 0.5 = 1.5° per step

At each step:
1. Compute the **target heading** (bearing to the next target waypoint)
2. Compute the **heading difference** using shortest-path normalization: `Δθ = atan2(sin(θ_target − θ_current), cos(θ_target − θ_current))` — this ensures the aircraft always turns the shorter way (e.g., 10° left instead of 350° right)
3. **Clamp** the heading change to ±maxStep, preventing unrealistic instantaneous course changes
4. **Move forward** by `v · Δt` meters in the new heading direction using spherical geometry
5. **Update altitude** linearly (climb) or via easeOutQuad (approach) based on elapsed time or distance

**Climb phase**: The turn rate is additionally modulated by an **easeInQuad ramp** so the aircraft flies straight immediately after liftoff and gradually begins turning toward the cruise waypoint — preventing an unrealistic sharp bank right after takeoff.

**Approach phase**: The aircraft steers toward the touchdown point with full turn rate available. When the remaining distance to touchdown is less than one time step of travel, the aircraft snaps to the touchdown position (arrival condition).

---

### 12. Easing Functions (Non-Linear Motion Curves)

The simulator employs several **easing functions** (borrowed from animation mathematics) to produce natural-feeling acceleration and deceleration curves:

| Function          | Formula                                   | Used For                                      |
| ----------------- | ----------------------------------------- | --------------------------------------------- |
| `easeInQuad`      | `t²`                                      | Takeoff liftoff altitude, climb turn rate ramp |
| `easeOutQuad`     | `t(2 − t)`                                | Landing descent altitude curve, flare pitch    |
| `easeInOutCubic`  | `4t³` (t<0.5), `1 − (−2t+2)³/2` (t≥0.5) | Descent altitude (smooth start and end)        |
| `lerp`            | `a + (b − a) · t`                         | All linear interpolation throughout the code   |

These functions map a linear progress parameter `t ∈ [0,1]` to a non-linear output, controlling how quantities (altitude, speed, pitch angle) evolve over time or distance:

- **easeOutQuad** for the approach altitude makes the plane descend steeply when far from the runway and gradually level off as it approaches — mimicking a real ILS glide slope.
- **easeInQuad** for the liftoff altitude makes the plane accelerate upward gradually, not popping off the runway.
- **easeInOutCubic** for the descent phase smoothly transitions from level cruise to descending and back to level at the approach entry — no sudden altitude changes.

---

### 13. 3D Orientation — Velocity-Derived Quaternion

Aircraft orientation (heading, pitch, roll) is computed in 3D using **quaternion mathematics**:

1. **Base orientation**: Cesium's `VelocityOrientationProperty` automatically computes a quaternion that aligns the aircraft model with its velocity vector. This handles heading and the natural pitch from climbing/descending.

2. **Pitch and roll adjustments** are applied as **axis-angle rotations** composed via quaternion multiplication:
   - Extract the local coordinate frame axes from the base quaternion using `Matrix3.fromQuaternion()` and `Matrix3.getColumn()`
   - Build pitch rotation around the local **right axis** (model Y-axis)
   - Build roll rotation around the local **forward axis** (model X-axis)
   - Compose: `Q_final = Q_roll × Q_pitch × Q_base`

---

### 14. Pitch (Flare) Dynamics

During the **landing** and **rollout** phases, the aircraft performs a **flare maneuver** — pitching the nose up to reduce sink rate before touchdown:

- **Flare start distance**: 1,500 m before the touchdown point
- **Maximum flare pitch**: 18° nose-up
- **Progress function**: easeOutQuad — pitch increases rapidly at first and levels off

```
t = 1 − clamp(distToTouchdown / 1500, 0, 1)
pitch = −lerp(0, 18°, easeOutQuad(t))
```

The pitch is negative because in the Cesium coordinate frame, a negative rotation around the right axis pitches the nose up.

---

### 15. Bank Angle (Roll) from Turn Rate

During airborne phases (climb, cruise, descent, landing), the aircraft **banks into turns** like a real airplane:

- The instantaneous **heading change rate** is estimated by sampling position at three points: `t`, `t + Δt`, `t + 2Δt` (where Δt = 0.5 s), computing the bearing difference, and dividing by Δt.
- The bank angle is proportional to the turn rate, clamped to ±25°:

```
bankFraction = clamp(turnRate / maxTurnRate, −1, +1)
roll = 25° × bankFraction
```

Where `maxTurnRate = 6°/s` corresponds to full bank. This produces realistic coordinated turns — faster turns create steeper banks, straight flight produces zero bank.

---

### 16. Hermite Polynomial Interpolation

Position samples (lat/lon/alt at specific times) are stored in Cesium's `SampledPositionProperty` and interpolated using **Hermite polynomial approximation** (degree 3). Unlike simple linear interpolation (which would produce jagged paths with visible corners), Hermite interpolation:

- Produces **C¹ continuous curves** (smooth position and velocity)
- Considers neighboring samples to compute tangent vectors
- Results in natural arcs through taxi turns and climb maneuvers

The interpolation is configured as:
```typescript
position.setInterpolationOptions({
  interpolationDegree: 3,
  interpolationAlgorithm: Cesium.HermitePolynomialApproximation,
});
```

---

### 17. Terrain & Elevation Model

The simulator uses **Cesium World Terrain** (Ion Asset ID 1) loaded asynchronously:

- **Vertex normals** enabled for realistic lighting and shading on terrain
- **Water mask** enabled for proper ocean rendering
- Airport **elevation above sea level (MSL)** is stored per airport and used to compute AGL (above ground level) for the speed profile

The altitude model uses two reference frames:
- **MSL (Mean Sea Level)**: Used for the speed profile and altitude display
- **AGL (Above Ground Level)**: `altitude_MSL − airport_elevation` — used by the speed profile function to determine the appropriate speed for the current height above the local terrain

---

### 18. Atmospheric & Visual Rendering

The 3D scene includes several physically-inspired visual effects:

- **Globe lighting** (`enableLighting: true`): The globe is lit based on the actual sun position, producing day/night shading
- **Sky atmosphere** rendering: Rayleigh and Mie scattering simulation for realistic sky colors at the horizon
- **Post-processing bloom**: Subtle glow effect (`brightness: −0.3`, `sigma: 3.0`) simulating atmospheric light scattering
- **FXAA anti-aliasing**: Full-screen anti-aliasing for smooth edges on terrain and models
- **Polyline glow trail**: The flight path renders as a glowing blue line (`PolylineGlowMaterialProperty`, `glowPower: 0.15`) trailing behind the aircraft

---

## Flight Phases

The simulation progresses through **11 distinct phases**, each with its own physics model:

| # | Phase           | Duration Method                      | Speed Profile                     | Altitude                                     |
|---|-----------------|--------------------------------------|-----------------------------------|----------------------------------------------|
| 1 | **Gate**        | Fixed (10 s)                         | Stationary (imperceptible drift)  | Ground (airport elevation)                   |
| 2 | **Taxi**        | Distance / 50 km/h                   | Constant 50 km/h                  | Ground                                       |
| 3 | **Runway Hold** | Fixed (3 s)                          | Stationary (drift along runway)   | Ground                                       |
| 4 | **Takeoff**     | 2 · runway_length / 300 km/h         | 0 → 300 km/h (constant accel.)   | Ground → +50 m liftoff                       |
| 5 | **Climb**       | Variable-speed simulation (0.5s Δt)  | 300 → 900 km/h (altitude-based)  | 50 m AGL → 10,000 m MSL over ~100 km         |
| 6 | **Cruise**      | Distance / 900 km/h                  | Constant 900 km/h                | Constant 10,000 m MSL                        |
| 7 | **Descent**     | Distance / 900 km/h                  | Constant 900 km/h                | 10,000 m → approach altitude (easeInOutCubic)|
| 8 | **Landing**     | Variable-speed simulation (0.5s Δt)  | 900 → 300 km/h (altitude-based)  | Approach altitude → ground (easeOutQuad)     |
| 9 | **Rollout**     | Analytical (ln-based integral)       | 300 → 50 km/h (linear decel.)   | Ground                                       |
|10 | **Arrival Taxi**| Distance / 50 km/h                   | Constant 50 km/h                 | Ground                                       |
|11 | **Arrival Gate**| Fixed (5 s)                          | Stationary                       | Ground                                       |

---

## Airport Database

The simulator includes **5 major world airports** with real GPS coordinates for gates, taxiways, runway thresholds, and liftoff points:

| Code | City         | Country   | Elevation (m MSL) | Runway Heading |
|------|--------------|-----------|--------------------|----------------|
| GDN  | Gdańsk       | Poland    | 165                | 112°           |
| WAW  | Warsaw       | Poland    | 132                | 110°           |
| NRT  | Tokyo Narita | Japan     | 80                 | 160°           |
| JFK  | New York     | USA       | 0                  | 310°           |
| SYD  | Sydney       | Australia | 30                 | 160°           |

Each airport has a full **departure route** (gate → taxi waypoints → runway threshold → runway waypoints → liftoff point) and a corresponding **arrival route** generated by reversing the departure route.

---

## Getting Started

### Prerequisites

- **Node.js** v18+ and **npm**
- A **Cesium Ion access token** (free at [cesium.com/ion](https://cesium.com/ion))

### Installation

```bash
# Clone the repository
git clone <repo-url>
cd symulator-cesium

# Install dependencies
npm install

# Configure the Cesium token
# Create or edit .env file:
echo "VITE_CESIUM_TOKEN=your_token_here" > .env

# Start the development server
npm run dev
```

### Build for Production

```bash
npm run build
npm run preview
```

---

## Configuration

| Variable             | File   | Description                                       |
| -------------------- | ------ | ------------------------------------------------- |
| `VITE_CESIUM_TOKEN`  | `.env` | Cesium Ion API token for terrain and imagery tiles |

### Key Physics Constants (in `flight.ts`)

| Constant              | Value      | Unit    | Description                           |
| --------------------- | ---------- | ------- | ------------------------------------- |
| `CRUISE_ALTITUDE`     | 10,000     | m MSL   | Target cruise altitude (~33,000 ft)   |
| `MAX_SPEED_KMH`       | 900        | km/h    | Maximum cruise speed                  |
| `MAX_GROUND_SPEED_KMH`| 50         | km/h    | Taxi and ground operations speed      |
| `LIFTOFF_SPEED_KMH`   | 300        | km/h    | Speed at liftoff and final approach   |
| `CLIMB_MID_SPEED_KMH` | 450        | km/h    | Mid-climb transition speed            |
| `CLIMB_DISTANCE_M`    | 100,000    | m       | Horizontal climb distance (~5.7° angle) |
| `APPROACH_DISTANCE_DEG`| 0.90      | degrees | Approach entry distance (~100 km)     |
| `MAX_TURN_RATE_RAD`   | 3          | °/s     | Standard Rate Turn limit              |
| `SIM_DT`              | 0.5        | s       | Simulation time step for turn sims    |
| `FLARE_START_DIST`    | 1,500      | m       | Distance before touchdown to begin flare |
| `FLARE_PITCH_RAD`     | 18         | °       | Maximum nose-up pitch during flare    |
| `BANK_MAX_RAD`        | 25         | °       | Maximum bank angle in turns           |

---

## License

This project is private. All rights reserved.
