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

let viewer: Cesium.Viewer;
let currentFlight: FlightResult | null = null;
let phaseUpdateInterval: number | null = null;
let arrivalTimeout: number | null = null;
let cameraTickListener: Cesium.Event.RemoveCallback | null = null;

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
const infoAltitude = document.getElementById("infoAltitude") as HTMLSpanElement;


function init(): void {
  viewer = initScene("cesiumContainer");

  populateDropdowns();
  wireEvents();

  // Set initial camera to a globe view
  viewer.camera.setView({
    destination: Cesium.Cartesian3.fromDegrees(18.4661, 54.3776, 15000000),
    orientation: {
      heading: 0,
      pitch: -Cesium.Math.PI_OVER_TWO,
      roll: 0,
    },
  });
}

//  Populate airport dropdowns 

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
        // here we increase the speed of the simulation
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
  flyButton.textContent = isSame ? "Select different airports" : "Start Flight";
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
    const speedKmh = Math.round(distanceMeters * 3.6 );
    infoSpeed.textContent = `${speedKmh} km/h`;
  } else {
    infoSpeed.textContent = "0 km/h";
  }

  // Altitude display
  if (pos1) {
    const carto = Cesium.Cartographic.fromCartesian(pos1);
    const altMeters = Math.max(0, Math.round(carto.height));
    if (altMeters >= 1000) {
      infoAltitude.textContent = `${(altMeters / 1000).toFixed(1)} km`;
    } else {
      infoAltitude.textContent = `${altMeters} m`;
    }
  } else {
    infoAltitude.textContent = "0 m";
  }
  });

  viewer.trackedEntity = currentFlight.entity;

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
    `Flight started: ${dep.city} → ${dest.city} (${Math.round(plan.distanceKm)} km, ${Math.round(plan.totalDuration)}s)`
  );
}

// Phase tracking 

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
    gate: "At Gate",
    taxi: "Taxiing",
    runway_hold: "Cleared for Takeoff",
    takeoff: "Taking Off!",
    climb: "Climbing",
    climb_align: "Climb Alignment Turn",
    cruise: "Cruising",
    descent_align: "Descent Alignment Turn",
    descent: "Descending",
    landing: "Landing",
    rollout: "Braking",
    arrival_taxi: "Taxiing to Gate",
    arrival_gate: "At Gate",
    arrived: "Arrived",
  };

  const phaseColors: Record<FlightPhase, string> = {
    preflight: "#94a3b8",
    gate: "#64748b",
    taxi: "#eab308",
    runway_hold: "#f97316",
    takeoff: "#ef4444",
    climb: "#3b82f6",
    climb_align: "#60a5fa",
    cruise: "#10b981",
    descent_align: "#a855f7",
    descent: "#8b5cf6",
    landing: "#f59e0b",
    rollout: "#f97316",
    arrival_taxi: "#eab308",
    arrival_gate: "#64748b",
    arrived: "#10b981",
  };

  phaseText.textContent = phaseLabels[phase];
  phaseDot.style.backgroundColor = phaseColors[phase];

  // Pulse animation on phase change
  phaseIndicator.classList.remove("phase-pulse");
  void phaseIndicator.offsetWidth; // force reflow
  phaseIndicator.classList.add("phase-pulse");
}

// Arrival handling

function handleArrival(): void {
  console.log("Flight arrived!");

  viewer.clock.shouldAnimate = false;
  cancelButton.textContent = "New Flight";

  // Stay on parked plane for 10 seconds, then reset
  arrivalTimeout = window.setTimeout(() => {
    resetFlight();
  }, 5000);
}

// Reset / Cancel flight

function resetFlight(): void {
  cleanupFlight();

  // Reset UI
  flyButton.style.display = "block";
  cancelButton.style.display = "none";
  cancelButton.textContent = "Cancel Flight";
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

init();
