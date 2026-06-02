/**
 * Flight Simulator — Main Application
 *
 * This is the entry point for the whole simulator. It sets up the 3D map,
 * connects all the buttons and dropdowns in the UI, and manages the flight
 * lifecycle (start, track phases, handle arrival, reset).
 *
 * Think of this as the "controller" that ties together the Cesium viewer
 * (the 3D globe) and the flight engine (flight.ts).
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


/**
 * Initialize everything when the page loads.
 * Sets up the 3D viewer, populates the airport dropdowns, and points
 * the camera at a nice globe view so the user sees something cool.
 */
function init(): void {
  viewer = initScene("cesiumContainer");

  populateDropdowns();
  wireEvents();

  // Start with a nice overview of the globe, centered roughly on Europe
  viewer.camera.setView({
    destination: Cesium.Cartesian3.fromDegrees(18.4661, 54.3776, 15000000),
    orientation: {
      heading: 0,
      pitch: -Cesium.Math.PI_OVER_TWO,
      roll: 0,
    },
  });
}

/**
 * Fill the departure and destination dropdowns with all available airports.
 * Sorted alphabetically by city name so they're easy to find.
 */
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

/**
 * Hook up all the event listeners — buttons, dropdowns, speed controls.
 * This runs once at init and connects the HTML elements to our functions.
 */
function wireEvents(): void {
  flyButton.addEventListener("click", startFlight);
  cancelButton.addEventListener("click", resetFlight);
  departureSelect.addEventListener("change", updateFlightInfo);
  destinationSelect.addEventListener("change", updateFlightInfo);

  // Speed control buttons (1x, 2x, 5x, 10x, etc.)
  // Each button has a data-speed attribute with the multiplier value
  document.querySelectorAll("[data-speed]").forEach((btn) => {
    btn.addEventListener("click", () => {
      const speed = parseFloat((btn as HTMLElement).dataset.speed || "1");
      if (viewer) {
        // This is how we speed up/slow down the simulation —
        // Cesium's clock multiplier controls how many sim-seconds
        // pass per real-world second
        viewer.clock.multiplier = speed;
      }
      // Update the active button styling
      document.querySelectorAll("[data-speed]").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
    });
  });
}

/**
 * Update the info panel (route, distance, duration) when the user
 * changes the departure or destination dropdown.
 * Also does a quick validation — you can't fly to the same airport!
 */
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

/**
 * The main function that kicks off a flight simulation.
 * Creates the flight plan, builds the 3D entity, starts the clock,
 * and switches the UI to "flying" mode.
 */
function startFlight(): void {
  const dep = AIRPORTS.find((a) => a.code === departureSelect.value) as Airport;
  const dest = AIRPORTS.find((a) => a.code === destinationSelect.value) as Airport;

  if (!dep || !dest || dep.code === dest.code) return;

  // Clean up any previous flight so we don't have stale entities
  cleanupFlight();

  const plan = planFlight(dep, dest);
  currentFlight = createFlight(viewer, plan);

  // Start the Cesium clock — this makes time actually advance
  viewer.clock.shouldAnimate = true;
  viewer.clock.multiplier = 1.0;

  // This tick listener runs every frame to update the speed/altitude display.
  // It calculates instantaneous speed by looking at position 1 second apart.
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

  // Altitude display — convert from Cesium's cartographic height
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

  // Make the camera follow the plane
  viewer.trackedEntity = currentFlight.entity;

  // Switch UI to "flying" mode — hide the fly button, show cancel + speed
  flyButton.style.display = "none";
  cancelButton.style.display = "block";
  speedControls.style.display = "flex";
  phaseIndicator.style.display = "flex";
  departureSelect.disabled = true;
  destinationSelect.disabled = true;

  // Reset speed button active state
  document.querySelectorAll("[data-speed]").forEach((b) => b.classList.remove("active"));
  document.querySelector('[data-speed="1"]')?.classList.add("active");

  // Start checking what phase we're in (climb, cruise, etc.)
  startPhaseTracking();

  console.log(
    `Flight started: ${dep.city} → ${dest.city} (${Math.round(plan.distanceKm)} km, ${Math.round(plan.totalDuration)}s)`
  );
}

/**
 * Poll the current flight phase every 200ms and update the UI.
 * When the phase changes (e.g. climb → cruise), it triggers a UI update
 * and a pulse animation to draw attention. When we reach "arrived",
 * it triggers the arrival handler.
 */
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
    cruise: "Cruising",
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
    cruise: "#10b981",
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

/**
 * Called when the flight reaches the "arrived" phase.
 * Stops the clock so the plane doesn't loop, changes the cancel button
 * text to "New Flight", and auto-resets after 5 seconds.
 */
function handleArrival(): void {
  console.log("Flight arrived!");

  // Stop the simulation clock so time doesn't keep running
  viewer.clock.shouldAnimate = false;
  cancelButton.textContent = "New Flight";

  // Wait 5 seconds so the user can see the plane at the gate,
  // then automatically reset to allow a new flight
  arrivalTimeout = window.setTimeout(() => {
    resetFlight();
  }, 5000);
}

/**
 * Reset the simulator back to its initial state.
 * Cleans up the current flight, restores the UI, and flies the camera
 * back to the nice globe overview.
 */
function resetFlight(): void {
  cleanupFlight();

  // Restore UI to "ready" state
  flyButton.style.display = "block";
  cancelButton.style.display = "none";
  cancelButton.textContent = "Cancel Flight";
  speedControls.style.display = "none";
  phaseIndicator.style.display = "none";
  departureSelect.disabled = false;
  destinationSelect.disabled = false;

  // Smooth camera transition back to the globe view
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

/**
 * Clean up all flight-related resources.
 * Clears intervals, timeouts, removes entities, stops the clock, etc.
 * This is called both when canceling a flight and when starting a new one
 * (to make sure we don't have leftover state from a previous flight).
 */
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
