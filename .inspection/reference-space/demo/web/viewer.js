import * as THREE from "./vendor/three.module.js";
import {
  mergeVertices,
  toCreasedNormals,
} from "./vendor/BufferGeometryUtils.js";

const quaternion = (q) => new THREE.Quaternion(q[1], q[2], q[3], q[0]);

export async function createViewer() {
  const response = await fetch("/scene");
  if (!response.ok) throw new Error("Could not load the robot scene.");
  const model = await response.json();
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#d8d7ce");
  const ambient = new THREE.HemisphereLight(0xffffff, 0x7c7567, 2);
  ambient.position.set(0, 0, 1);
  scene.add(ambient);
  const light = new THREE.DirectionalLight(0xfff5e7, 2.2);
  light.position.set(0.3, -0.4, 1.2);
  light.target.position.set(0.2, 0, 0);
  light.castShadow = true;
  light.shadow.mapSize.set(1024, 1024);
  Object.assign(light.shadow.camera, {
    left: -0.6,
    right: 0.6,
    top: 0.6,
    bottom: -0.6,
    near: 0.01,
    far: 4,
  });
  light.shadow.bias = -0.0001;
  light.shadow.normalBias = 0.0003;
  scene.add(light, light.target);
  const fill = new THREE.DirectionalLight(0xffffff, 0.8);
  fill.position.set(-0.2, 0.4, 1);
  scene.add(fill);

  const bodies = Array.from({ length: model.bodies }, () => {
    const group = new THREE.Group();
    scene.add(group);
    return group;
  });
  const meshes = new Map();
  for (const [id, data] of Object.entries(model.meshes)) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(data.vertices, 3),
    );
    geometry.setIndex(data.faces);
    // STL normals smooth across mechanical edges, exposing the triangulation.
    // Work in millimetres so the utility's position hash cannot join nearby surfaces.
    geometry.scale(1000, 1000, 1000);
    const creased = toCreasedNormals(geometry, Math.PI / 5);
    const display = mergeVertices(creased, 1e-6);
    display.scale(0.001, 0.001, 0.001);
    geometry.dispose();
    creased.dispose();
    meshes.set(id, display);
  }
  const wood = new THREE.TextureLoader().load("/demo/wood.png");
  wood.colorSpace = THREE.SRGBColorSpace;
  const materials = new Map();
  for (const geom of model.geoms) {
    let geometry;
    const [x, y, z] = geom.size;
    switch (geom.type) {
      case "mesh":
        geometry = meshes.get(geom.mesh);
        break;
      case "box":
        geometry = new THREE.BoxGeometry(2 * x, 2 * y, 2 * z);
        break;
      case "plane":
        geometry = new THREE.PlaneGeometry(
          2 * Math.max(x, 2),
          2 * Math.max(y, 2),
        );
        break;
      case "sphere":
        geometry = new THREE.SphereGeometry(x, 20, 12);
        break;
      case "ellipsoid":
        geometry = new THREE.SphereGeometry(1, 20, 12);
        geometry.scale(x, y, z);
        break;
      case "cylinder":
        geometry = new THREE.CylinderGeometry(x, x, 2 * y, 24);
        geometry.rotateX(Math.PI / 2);
        break;
      case "capsule":
        geometry = new THREE.CapsuleGeometry(x, 2 * y, 6, 16);
        geometry.rotateX(Math.PI / 2);
        break;
      default:
        continue;
    }
    const materialKey = JSON.stringify([geom.rgba, geom.wood]);
    if (!materials.has(materialKey)) {
      const [r, g, b, a] = geom.rgba;
      materials.set(
        materialKey,
        new THREE.MeshStandardMaterial({
          color: new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace),
          roughness: 0.65,
          metalness: 0.03,
          opacity: a,
          transparent: a < 1,
          map: geom.wood ? wood : null,
        }),
      );
    }
    const mesh = new THREE.Mesh(geometry, materials.get(materialKey));
    mesh.position.fromArray(geom.pos);
    mesh.quaternion.copy(quaternion(geom.quat));
    mesh.castShadow = geom.type !== "plane";
    mesh.receiveShadow = true;
    bodies[geom.body].add(mesh);
  }

  const views = ["front", "wrist", "top"].map((name) => {
    const canvas = document.getElementById(name);
    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(name === "top" ? 1 : Math.min(devicePixelRatio, 1.5));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.1;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    const camera = new THREE.PerspectiveCamera(60, 4 / 3, 0.003, 15);
    let observer = null;
    if (name === "top") renderer.setSize(160, 120, false);
    else {
      observer = new ResizeObserver(() => {
        const width = Math.max(1, canvas.parentElement.clientWidth);
        renderer.setSize(width, Math.round((width * 3) / 4), false);
      });
      observer.observe(canvas.parentElement);
    }
    return { name, canvas, camera, renderer, observer };
  });

  let previous = null,
    current = null,
    active = true,
    visionFrame = null,
    visionRequested = false;
  let frameCount = 0,
    fpsStart = performance.now();
  const p0 = new THREE.Vector3(),
    p1 = new THREE.Vector3();
  const q0 = new THREE.Quaternion(),
    q1 = new THREE.Quaternion();
  function transform(object, first, second, alpha) {
    p0.fromArray(first);
    p1.fromArray(second);
    object.position.copy(p0).lerp(p1, alpha);
    q0.set(first[4], first[5], first[6], first[3]);
    q1.set(second[4], second[5], second[6], second[3]);
    object.quaternion.copy(q0).slerp(q1, alpha);
  }
  function draw(now) {
    requestAnimationFrame(draw);
    if (!active || !current || document.hidden) return;
    const alpha = previous
      ? THREE.MathUtils.clamp(
          (now - 40 - previous.received) /
            Math.max(1, current.received - previous.received),
          0,
          1,
        )
      : 1;
    const old = previous || current;
    bodies.forEach((body, i) =>
      transform(body, old.bodies[i], current.bodies[i], alpha),
    );
    for (const view of views) {
      const first = old.cameras[view.name],
        second = current.cameras[view.name];
      transform(
        view.camera,
        [...first.pos, ...first.quat],
        [...second.pos, ...second.quat],
        alpha,
      );
      if (view.camera.fov !== second.fovy) {
        view.camera.fov = second.fovy;
        view.camera.updateProjectionMatrix();
      }
      view.renderer.render(scene, view.camera);
      if (view.name === "top" && visionRequested) {
        const gl = view.renderer.getContext();
        const pixels = new Uint8Array(view.canvas.width * view.canvas.height * 4);
        gl.readPixels(
          0,
          0,
          view.canvas.width,
          view.canvas.height,
          gl.RGBA,
          gl.UNSIGNED_BYTE,
          pixels,
        );
        let binary = "";
        for (let start = 0; start < pixels.length; start += 0x8000) {
          binary += String.fromCharCode(...pixels.subarray(start, start + 0x8000));
        }
        visionFrame = btoa(binary);
        visionRequested = false;
      }
    }
    frameCount++;
    if (now - fpsStart >= 1000) {
      document.getElementById("frame-rate").textContent =
        `${Math.round((frameCount * 1000) / (now - fpsStart))} FPS`;
      frameCount = 0;
      fpsStart = now;
    }
  }
  requestAnimationFrame(draw);
  return {
    update(frame) {
      previous = current;
      current = {
        bodies: frame.bodies,
        cameras: frame.cameras,
        received: performance.now(),
      };
      active = true;
      for (const view of views) {
        view.canvas.hidden = false;
        const poster = document.getElementById(`${view.name}-poster`);
        if (poster) poster.hidden = true;
      }
      document.getElementById("top-prompt").hidden = true;
    },
    getVisionFrame() {
      visionRequested = true;
      return visionFrame;
    },
    pause() {
      active = false;
      previous = null;
      current = null;
      visionFrame = null;
      visionRequested = false;
    },
  };
}
