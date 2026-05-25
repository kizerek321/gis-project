/**
 * Cesium scene initialization — viewer, terrain, atmosphere.
 */
import * as Cesium from "cesium";

let viewer: Cesium.Viewer;

export function initScene(containerId: string): Cesium.Viewer {
  // Set the access token from environment
  const token = import.meta.env.VITE_CESIUM_TOKEN as string;
  if (!token) {
    console.error("VITE_CESIUM_TOKEN is not set in .env");
  }
  Cesium.Ion.defaultAccessToken = token;

  viewer = new Cesium.Viewer(containerId, {
    timeline: true,
    animation: true,
    baseLayerPicker: false,
    geocoder: false,
    shadows: false,
    homeButton: false,
    sceneModePicker: false,
    navigationHelpButton: false,
    fullscreenButton: false,
    vrButton: false,
    infoBox: false,
    selectionIndicator: false,
  });

  // Load Cesium World Terrain for realistic ground
  setupTerrain();

  // Atmosphere & lighting
  const scene = viewer.scene;
  scene.globe.enableLighting = true;

  if (scene.skyAtmosphere) {
    scene.skyAtmosphere.brightnessShift = 0.0;
    scene.skyAtmosphere.saturationShift = 0.0;
  }

  // Post-processing — subtle bloom for atmosphere glow
  const bloom = scene.postProcessStages.bloom;
  bloom.enabled = true;
  bloom.uniforms.brightness = -0.3;
  bloom.uniforms.delta = 1.0;
  bloom.uniforms.sigma = 3.0;

  scene.postProcessStages.fxaa.enabled = true;

  // Clock defaults
  viewer.clock.shouldAnimate = false;
  viewer.clock.multiplier = 1.0;

  return viewer;
}

async function setupTerrain(): Promise<void> {
  try {
    const terrainProvider = await Cesium.CesiumTerrainProvider.fromIonAssetId(1, {
      requestVertexNormals: true,
      requestWaterMask: true,
    });
    viewer.terrainProvider = terrainProvider;
    console.log("Cesium World Terrain loaded");
  } catch (err) {
    console.warn("Terrain loading failed, using default:", err);
  }
}

export function getViewer(): Cesium.Viewer {
  return viewer;
}
