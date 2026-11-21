import { createViewer } from "./viewer.js?v=creases-1";
let viewer = null;
let lastMotionLimit = false;
const $ = (id) => document.getElementById(id);
let socket = null,
  timer = null,
  connected = false,
  pending = null,
  latest = null,
  autoEnabled = false,
  lastVisionSent = 0;
const pressed = new Set(),
  pointerAxes = new Map();
const names = [
  "Shoulder pan",
  "Shoulder lift",
  "Elbow flex",
  "Wrist flex",
  "Wrist roll",
  "Gripper",
];
const defaultLimits = [
  [-110, -100, -97, -95, -157, 0],
  [110, 100, 97, 95, 163, 100],
];
const keymap = {
  KeyW: [0, 1],
  KeyS: [0, -1],
  KeyA: [1, 1],
  KeyD: [1, -1],
  KeyQ: [2, 1],
  KeyE: [2, -1],
  ArrowUp: [3, -1],
  ArrowDown: [3, 1],
  ArrowLeft: [4, 1],
  ArrowRight: [4, -1],
};
const sliders = names.map((name, i) => {
  const item = document.createElement("div");
  item.className = "joint";
  item.innerHTML = `<label for="joint-${i}">${name}<output id="value-${i}">—</output></label><input type="range" id="joint-${i}" min="${defaultLimits[0][i]}" max="${defaultLimits[1][i]}" step="0.5" value="0" disabled aria-label="${name} target">`;
  $("joint-sliders").append(item);
  const slider = item.querySelector("input");
  slider.addEventListener("input", () => {
    $(`value-${i}`).textContent = `${slider.value}${i === 5 ? "%" : "°"}`;
    pending = {
      command: "joints",
      target: sliders.map((s) => Number(s.value)),
    };
  });
  return slider;
});
function setStatus(text, error = false) {
  $("status").textContent = text;
  $("status").classList.toggle("error", error);
}
function clearKeys() {
  pressed.clear();
  pointerAxes.clear();
  document
    .querySelectorAll(".pressed")
    .forEach((el) => el.classList.remove("pressed"));
}
function axes() {
  const values = [0, 0, 0, 0, 0, 0];
  for (const code of pressed) {
    const pair = keymap[code];
    if (pair) values[pair[0]] += pair[1];
  }
  for (const pair of pointerAxes.values()) values[pair[0]] += pair[1];
  return values.map((v) => Math.max(-1, Math.min(1, v)));
}
function sendNext() {
  if (!connected || socket?.readyState !== WebSocket.OPEN) return;
  const message = pending || { command: "step" };
  pending = null;
  message.axes = axes();
  message.precision =
    $("precision").checked ||
    pressed.has("ShiftLeft") ||
    pressed.has("ShiftRight");
  if (autoEnabled && performance.now() - lastVisionSent >= 100) {
    const vision = viewer?.getVisionFrame();
    if (vision) message.vision = vision;
    lastVisionSent = performance.now();
  }
  if (socket.bufferedAmount > 8192) return;
  message.visible = !document.hidden;
  socket.send(JSON.stringify(message));
}
function setEnabled(value) {
  ["reset", "shuffle", "home", "gripper", "auto-sort"].forEach(
    (id) => ($(id).disabled = !value),
  );
  sliders.forEach((el) => (el.disabled = !value));
}
function render(frame) {
  if (frame.error) {
    setStatus(frame.error, true);
    socket?.close();
    return;
  }
  if (latest?.success && !frame.success) setStatus("");
  latest = frame;
  $("cube-count").innerHTML = `${frame.cubes}<span>/ 4</span>`;
  viewer.update(frame);
  sliders.forEach((slider, i) => {
    slider.min = frame.limits[0][i];
    slider.max = frame.limits[1][i];
    if (document.activeElement !== slider && pending?.command !== "joints")
      slider.value = frame.target[i];
    $(`value-${i}`).textContent =
      `${Number(slider.value).toFixed(0)}${i === 5 ? "%" : "°"}`;
  });
  $("gripper-label").textContent =
    frame.target[5] > 50 ? "Close gripper" : "Open gripper";
  $("auto-sort").textContent = frame.auto_running
    ? "Stop auto sort"
    : "Auto sort by color";
  if (document.activeElement !== $("seed")) $("seed").value = frame.seed;
  if (frame.motion_limit) {
    setStatus(
      frame.motion_limit === "joint"
        ? "This joint is at its limit. Reverse the wrist control, or use Home pose."
        : "At a reach or contact limit. Try another direction, or use Home pose.",
    );
  } else if (lastMotionLimit) {
    setStatus("");
  }
  lastMotionLimit = Boolean(frame.motion_limit);
  if (frame.success) {
    clearKeys();
    setStatus("All four cubes are in their matching boxes. Resetting…");
  } else if (frame.ended)
    setStatus(
      "This session reached its time limit. Reset the scene to continue.",
    );
  if (frame.auto_running) setStatus(frame.auto_status);
  else if (autoEnabled) {
    autoEnabled = false;
    setStatus(frame.auto_status);
  }
}
async function connect() {
  if (
    socket &&
    [WebSocket.CONNECTING, WebSocket.OPEN].includes(socket.readyState)
  )
    return;
  $("start").disabled = true;
  $("start").textContent = "Starting simulation…";
  $("connection").textContent = "STARTING SIMULATION";
  setStatus("Starting simulation…");
  try {
    viewer ||= await createViewer();
  } catch (error) {
    setStatus(`Could not start browser rendering: ${error.message}`, true);
    $("connection").textContent = "READY";
    $("start").disabled = false;
    $("start").textContent = "Start simulation";
    return;
  }
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${protocol}//${location.host}/ws`);
  socket.onmessage = (event) => {
    const frame = JSON.parse(event.data);
    if (frame.error) {
      render(frame);
      return;
    }
    if (!connected) {
      connected = true;
      $("start-overlay").hidden = true;
      $("live-dot").classList.add("connected");
      $("connection").textContent = "LIVE";
      $("disconnect").hidden = false;
      setEnabled(true);
      $("viewport").focus();

      timer = setInterval(sendNext, 50);
      setStatus("");
    }
    render(frame);
  };
  socket.onerror = () =>
    setStatus(
      "Could not connect to the simulator. Check the server and try again.",
      true,
    );
  socket.onclose = () => {
    connected = false;
    autoEnabled = false;
    clearKeys();
    clearInterval(timer);
    pending = null;
    setEnabled(false);
    $("start-overlay").hidden = false;
    $("start").disabled = false;
    $("start").innerHTML = "Start simulation <span>→</span>";
    $("live-dot").classList.remove("connected");
    viewer?.pause();
    $("connection").textContent = "SESSION ENDED";
    $("disconnect").hidden = true;
    $("frame-rate").textContent = "— FPS";
  };
}
$("start").addEventListener("click", connect);
$("disconnect").addEventListener("click", () => {
  socket?.close();
  setStatus("Session ended. Start again whenever you like.");
});
$("reset").addEventListener("click", () => {
  autoEnabled = false;
  clearKeys();
  pending = { command: "reset", seed: Number($("seed").value) || 0 };
  setStatus("Resetting to the same seeded tabletop.");
});
$("shuffle").addEventListener("click", () => {
  autoEnabled = false;
  clearKeys();
  pending = { command: "reset", seed: Math.floor(Math.random() * 1000000) };
  setStatus("Scene reset with a new seed.");
});
$("home").addEventListener("click", () => {
  autoEnabled = false;
  clearKeys();
  pending = { command: "home" };
  setStatus("Returning to the home pose.");
});
$("gripper").addEventListener("click", () => {
  autoEnabled = false;
  pending = { command: "gripper" };
});
$("auto-sort").addEventListener("click", () => {
  clearKeys();
  autoEnabled = !autoEnabled;
  pending = { command: autoEnabled ? "auto_start" : "auto_stop" };
  setStatus(autoEnabled ? "Starting color sorting…" : "Stopping automatic sorting.");
});
$("viewport").addEventListener(
  "focus",
  () =>
    ($("keyboard-state").textContent =
      "KEYBOARD ACTIVE · hold Shift for precision"),
);
$("viewport").addEventListener("blur", () => {
  clearKeys();
  $("keyboard-state").textContent = "Click the view to focus keyboard controls";
});
window.addEventListener("keydown", (event) => {
  if (!connected || document.activeElement !== $("viewport")) return;
  if (keymap[event.code] || event.code === "Space") {
    event.preventDefault();
    if (event.code === "Space") {
      if (!event.repeat) pending = { command: "gripper" };
    } else {
      pressed.add(event.code);
      highlight();
    }
  } else if (event.code.startsWith("Shift")) pressed.add(event.code);
});
window.addEventListener("keyup", (event) => {
  pressed.delete(event.code);
  highlight();
});
window.addEventListener("blur", clearKeys);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) clearKeys();
});
window.addEventListener("pagehide", () => socket?.close());
function highlight() {
  document.querySelectorAll("[data-axis]").forEach((button) => {
    const active = [...pressed].some(
      (code) =>
        keymap[code]?.[0] === Number(button.dataset.axis) &&
        keymap[code]?.[1] === Number(button.dataset.dir),
    );
    button.classList.toggle(
      "pressed",
      active ||
        [...pointerAxes.values()].some(
          (pair) =>
            pair[0] === Number(button.dataset.axis) &&
            pair[1] === Number(button.dataset.dir),
        ),
    );
  });
}
document.querySelectorAll("[data-axis]").forEach((button) => {
  button.addEventListener("pointerdown", (event) => {
    if (!connected) return;
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    pointerAxes.set(event.pointerId, [
      Number(button.dataset.axis),
      Number(button.dataset.dir),
    ]);
    highlight();
  });
  const release = (event) => {
    pointerAxes.delete(event.pointerId);
    highlight();
  };
  button.addEventListener("pointerup", release);
  button.addEventListener("pointercancel", release);
  button.addEventListener("lostpointercapture", release);
});
