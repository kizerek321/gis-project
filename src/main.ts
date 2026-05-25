/**
 * Flight Simulator — Main Application
 *
 * Bootstraps the Cesium scene, wires up the UI, and orchestrates
 * flight simulation with realistic departure camera choreography.
 */
import * as Cesium from "cesium";
import "cesium/Build/Cesium/Widgets/widgets.css";
import "./style.css";

import { AIRPORTS, getDistanceKm } from "./airports";
import type { Airport } from "./airports";
import { initScene } from "./scene";
import { planFlight, createFlight } from "./flight";
import type { FlightResult, FlightPhase } from "./flight";

// ─── State ──────────────────────────────────────────────────

let viewer: Cesium.Viewer;
let currentFlight: FlightResult | null = null;
let phaseUpdateInterval: number | null = null;
let arrivalTimeout: number | null = null;
let cameraTickListener: Cesium.Event.RemoveCallback | null = null;

// ─── DOM References ─────────────────────────────────────────

const departureSelect = document.getElementById("departureSelect") as HTMLSelectElement;
const destinationSelect = document.getElementById("destinationSelect") as HTMLSelectElement;
const flyButton = document.getElementById("flyButton") as HTMLButtonElement;
const infoRoute = document.getElementById("infoRoute") as HTMLSpanElement;
const infoDistance = document.getElementById("infoDistance") as HTMLSpanElement;
const infoDuration = document.getElementById("infoDuration") as HTMLSpanElement;
const phaseIndicator = document.getElementById("phaseIndicator") as HTMLDivElement;
const phaseDot = document.getElementById("phaseDot") as HTMLSpanElement;
const phaseText = document.getElementById("phaseText") as HTMLSpanElement;
const speedControls = document.getElementById("speedControls") as HTMLDivElement;
const cancelButton = document.getElementById("cancelButton") as HTMLButtonElement;
const infoSpeed = document.getElementById("infoSpeed") as HTMLSpanElement;

// ─── Initialize ─────────────────────────────────────────────

function init(): void {
  viewer = initScene("cesiumContainer");

  populateDropdowns();
  wireEvents();

  // Set initial camera to a nice globe view
  viewer.camera.setView({
    destination: Cesium.Cartesian3.fromDegrees(18.4661, 54.3776, 15000000),
    orientation: {
      heading: 0,
      pitch: -Cesium.Math.PI_OVER_TWO,
      roll: 0,
    },
  });
}

// ─── Populate airport dropdowns ─────────────────────────────

function populateDropdowns(): void {
  const sorted = [...AIRPORTS].sort((a, b) => a.city.localeCompare(b.city));

  for (const airport of sorted) {
    const label = `${airport.city} (${airport.code}) — ${airport.country}`;

    const optDep = document.createElement("option");
    optDep.value = airport.code;
    optDep.textContent = label;
    departureSelect.appendChild(optDep);

    const optDest = document.createElement("option");
    optDest.value = airport.code;
    optDest.textContent = label;
    destinationSelect.appendChild(optDest);
  }

  // Default selection: Gdańsk → Tokyo Narita
  departureSelect.value = "GDN";
  destinationSelect.value = "NRT";

  updateFlightInfo();
}

// ─── Event wiring ───────────────────────────────────────────

function wireEvents(): void {
  flyButton.addEventListener("click", startFlight);
  cancelButton.addEventListener("click", resetFlight);
  departureSelect.addEventListener("change", updateFlightInfo);
  destinationSelect.addEventListener("change", updateFlightInfo);

  // Speed control buttons
  document.querySelectorAll("[data-speed]").forEach((btn) => {
    btn.addEventListener("click", () => {
      const speed = parseFloat((btn as HTMLElement).dataset.speed || "1");
      if (viewer) {
        viewer.clock.multiplier = speed;
      }
      document.querySelectorAll("[data-speed]").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
    });
  });
}

// ─── Update flight info display ─────────────────────────────

function updateFlightInfo(): void {
  const dep = AIRPORTS.find((a) => a.code === departureSelect.value);
  const dest = AIRPORTS.find((a) => a.code === destinationSelect.value);

  if (!dep || !dest) return;

  const distance = getDistanceKm(dep, dest);
  const plan = planFlight(dep, dest);

  infoRoute.textContent = `${dep.code} → ${dest.code}`;
  infoDistance.textContent = `${Math.round(distance).toLocaleString()} km`;

  const mins = Math.floor(plan.totalDuration / 60);
  const secs = Math.round(plan.totalDuration % 60);
  infoDuration.textContent = mins > 0 ? `${mins}m ${secs}s` : `${secs}s`;

  // Validate — can't fly to same airport
  const isSame = dep.code === dest.code;
  flyButton.disabled = isSame;
  flyButton.textContent = isSame ? "Select different airports" : "✈️ Start Flight";
}

// ─── Start flight simulation ────────────────────────────────

function startFlight(): void {
  const dep = AIRPORTS.find((a) => a.code === departureSelect.value) as Airport;
  const dest = AIRPORTS.find((a) => a.code === destinationSelect.value) as Airport;

  if (!dep || !dest || dep.code === dest.code) return;

  // Clean up any previous flight
  cleanupFlight();

  const plan = planFlight(dep, dest);
  currentFlight = createFlight(viewer, plan);

  // Start clock
  viewer.clock.shouldAnimate = true;
  viewer.clock.multiplier = 1.0;
  viewer.clock.onTick.addEventListener((clock) => {
  if (!currentFlight || !currentFlight.entity.position) return;

  const currentTime = clock.currentTime;
  const nextTime = Cesium.JulianDate.addSeconds(currentTime, 1, new Cesium.JulianDate());

  const pos1 = currentFlight.entity.position.getValue(currentTime);
  const pos2 = currentFlight.entity.position.getValue(nextTime);

  if (pos1 && pos2) {
    const distanceMeters = Cesium.Cartesian3.distance(pos1, pos2);
    // Przeliczenie m/s na km/h
    infoSpeed.textContent = `${Math.round(distanceMeters * 3.6)} km/h`;
  } else {
    infoSpeed.textContent = "0 km/h";
  }
  });

  // Camera setup: route-based or simple
  if (currentFlight.departureCamera) {
    startDepartureChoreography(currentFlight);
  } else {
    // Simple: immediately track the entity
    viewer.trackedEntity = currentFlight.entity;
  }

  // UI state: flying
  flyButton.style.display = "none";
  cancelButton.style.display = "block";
  speedControls.style.display = "flex";
  phaseIndicator.style.display = "flex";
  departureSelect.disabled = true;
  destinationSelect.disabled = true;

  // Reset speed button active state
  document.querySelectorAll("[data-speed]").forEach((b) => b.classList.remove("active"));
  document.querySelector('[data-speed="1"]')?.classList.add("active");

  // Start phase tracking
  startPhaseTracking();

  console.log(
    `🛫 Flight started: ${dep.city} → ${dest.city} (${Math.round(plan.distanceKm)} km, ${Math.round(plan.totalDuration)}s)`
  );
}

// ─── Departure Camera Choreography ──────────────────────────

/**
 * Manages camera during gate hold and taxi phases:
 * 1. Camera looks at the FRONT of the plane for gateHoldTime seconds
 * 2. Camera flies to the REAR of the plane over 2 seconds
 * 3. Switches to entity tracking for the rest of the flight
 */
function startDepartureChoreography(flight: FlightResult): void {
  const cam = flight.departureCamera!;
  const elev = flight.plan.departure.elevation;
  const heading = cam.initialHeading;
  const gateHoldTime = flight.plan.timing.gate;

  // ── 1. Position camera in FRONT of the plane ──────────
  // Camera ahead of the plane (in the direction the plane faces), looking back
  const FRONT_DIST = 50;  // meters ahead
  const FRONT_ALT = 8;    // meters above ground
  const metersPerDegLat = 111320;
  const metersPerDegLon = 111320 * Math.cos(cam.gatePosition.lat * Math.PI / 180);

  const frontLat = cam.gatePosition.lat + (FRONT_DIST * Math.cos(heading)) / metersPerDegLat;
  const frontLon = cam.gatePosition.lon + (FRONT_DIST * Math.sin(heading)) / metersPerDegLon;

  viewer.camera.setView({
    destination: Cesium.Cartesian3.fromDegrees(frontLon, frontLat, elev + FRONT_ALT),
    orientation: {
      heading: (heading + Math.PI) % (2 * Math.PI), // looking back toward the plane
      pitch: Cesium.Math.toRadians(-8),
      roll: 0,
    },
  });

  // ── 2. Use clock tick to transition camera at the right sim time ──
  let choreographyState: "front" | "transitioning" | "tracking" = "front";

  cameraTickListener = viewer.clock.onTick.addEventListener((clock) => {
    if (!currentFlight) return;
    const elapsed = Cesium.JulianDate.secondsDifference(clock.currentTime, flight.startTime);

    if (choreographyState === "front" && elapsed >= gateHoldTime) {
      choreographyState = "transitioning";

      // ── Fly camera to BEHIND the plane ────────────────
      const REAR_DIST = 80;  // meters behind
      const REAR_ALT = 30;   // meters above ground

      const rearLat = cam.gatePosition.lat - (REAR_DIST * Math.cos(heading)) / metersPerDegLat;
      const rearLon = cam.gatePosition.lon - (REAR_DIST * Math.sin(heading)) / metersPerDegLon;

      viewer.camera.flyTo({
        destination: Cesium.Cartesian3.fromDegrees(rearLon, rearLat, elev + REAR_ALT),
        orientation: {
          heading: heading, // looking toward the plane (in its forward direction)
          pitch: Cesium.Math.toRadians(-15),
          roll: 0,
        },
        duration: 2,
        complete: () => {
          choreographyState = "tracking";
          // Switch to tracked entity for the rest of the flight
          if (currentFlight) {
            viewer.trackedEntity = currentFlight.entity;
          }
          // Remove this listener
          if (cameraTickListener) {
            cameraTickListener();
            cameraTickListener = null;
          }
        },
      });
    }
  });
}

// ─── Phase tracking ─────────────────────────────────────────

function startPhaseTracking(): void {
  if (phaseUpdateInterval !== null) {
    clearInterval(phaseUpdateInterval);
  }

  let lastPhase: FlightPhase = "preflight";

  phaseUpdateInterval = window.setInterval(() => {
    if (!currentFlight) return;

    const phase = currentFlight.getPhase(viewer.clock.currentTime);

    if (phase !== lastPhase) {
      lastPhase = phase;
      updatePhaseUI(phase);

      if (phase === "arrived") {
        handleArrival();
      }
    }
  }, 200);
}

function updatePhaseUI(phase: FlightPhase): void {
  const phaseLabels: Record<FlightPhase, string> = {
    preflight: "Pre-flight",
    gate: "🅿️ At Gate",
    taxi: "🚕 Taxiing",
    runway_hold: "🛫 Cleared for Takeoff",
    takeoff: "🛫 Taking Off!",
    climb: "📈 Climbing",
    cruise: "✈️ Cruising",
    descent: "📉 Descending",
    landing: "🛬 Landing",
    arrived: "✅ Arrived",
  };

  const phaseColors: Record<FlightPhase, string> = {
    preflight: "#94a3b8",
    gate: "#64748b",
    taxi: "#eab308",
    runway_hold: "#f97316",
    takeoff: "#ef4444",
    climb: "#3b82f6",
    cruise: "#10b981",
    descent: "#8b5cf6",
    landing: "#f59e0b",
    arrived: "#10b981",
  };

  phaseText.textContent = phaseLabels[phase];
  phaseDot.style.backgroundColor = phaseColors[phase];

  // Pulse animation on phase change
  phaseIndicator.classList.remove("phase-pulse");
  void phaseIndicator.offsetWidth; // force reflow
  phaseIndicator.classList.add("phase-pulse");
}

// ─── Arrival handling ───────────────────────────────────────

function handleArrival(): void {
  console.log("🛬 Flight arrived!");

  viewer.clock.shouldAnimate = false;
  cancelButton.textContent = "New Flight";

  // Stay on parked plane for 10 seconds, then reset
  arrivalTimeout = window.setTimeout(() => {
    resetFlight();
  }, 10000);
}

// ─── Reset / Cancel flight ──────────────────────────────────

function resetFlight(): void {
  cleanupFlight();

  // Reset UI
  flyButton.style.display = "block";
  cancelButton.style.display = "none";
  cancelButton.textContent = "✕ Cancel Flight";
  speedControls.style.display = "none";
  phaseIndicator.style.display = "none";
  departureSelect.disabled = false;
  destinationSelect.disabled = false;

  // Reset camera to globe view
  viewer.camera.flyTo({
    destination: Cesium.Cartesian3.fromDegrees(18.4661, 54.3776, 15000000),
    orientation: {
      heading: 0,
      pitch: -Cesium.Math.PI_OVER_TWO,
      roll: 0,
    },
    duration: 2,
  });

  updateFlightInfo();
}

function cleanupFlight(): void {
  if (phaseUpdateInterval !== null) {
    clearInterval(phaseUpdateInterval);
    phaseUpdateInterval = null;
  }

  if (arrivalTimeout !== null) {
    clearTimeout(arrivalTimeout);
    arrivalTimeout = null;
  }

  if (cameraTickListener) {
    cameraTickListener();
    cameraTickListener = null;
  }

  viewer.trackedEntity = undefined;
  viewer.entities.removeAll();
  viewer.clock.shouldAnimate = false;
  viewer.clock.multiplier = 1.0;

  currentFlight = null;
}

// ─── Boot ───────────────────────────────────────────────────

init();
