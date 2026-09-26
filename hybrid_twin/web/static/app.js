import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";

const $ = (id) => document.getElementById(id);
const SEND_HZ = 30;
const LIFT_KEY_LEAD = 50; // mm ahead of measured while R/F is held (AlohaMiniClient max lead)
const LIFT_DONE_MM = 4;
const ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"];

// ---------------------------------------------------------------- 3D scene
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1216);
scene.fog = new THREE.Fog(0x0f1216, 8, 24);

const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.01, 100);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true;
$("viewport").appendChild(renderer.domElement);

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1d22, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(3, 6, 2);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -3, right: 3, top: 3, bottom: -3 });
scene.add(sun, sun.target);

// 0.5 m checker tiles with edge lines: gives clear motion cues while driving.
function floorTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 256;
  const g = c.getContext("2d");
  g.fillStyle = "#1d242d"; g.fillRect(0, 0, 256, 256);
  g.fillStyle = "#232b35"; g.fillRect(0, 0, 128, 128); g.fillRect(128, 128, 128, 128);
  g.strokeStyle = "#3a4654"; g.lineWidth = 3; g.strokeRect(0, 0, 256, 256);
  g.beginPath(); g.moveTo(128, 0); g.lineTo(128, 256); g.moveTo(0, 128); g.lineTo(256, 128); g.stroke();
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(60, 60); // 60 m floor / 1 m per texture = 0.5 m tiles
  t.anisotropy = renderer.capabilities.getMaxAnisotropy();
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}
const floor = new THREE.Mesh(new THREE.PlaneGeometry(60, 60),
  new THREE.MeshStandardMaterial({ map: floorTexture(), roughness: 0.95 }));
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);

// Driven path on the floor.
const TRAIL_MAX = 4000;
const trailPos = new Float32Array(TRAIL_MAX * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
trailGeo.setDrawRange(0, 0);
scene.add(new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xe8a33d, transparent: true, opacity: 0.6 })));
let trailN = 0;

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.49;
function resetView() {
  camera.position.set(1.9, 1.5, 1.9);
  controls.target.set(0, 0.55, 0);
}
resetView();

// URDF is Z-up; three.js is Y-up. World point (x, y, z) -> three (x, z, -y).
const toThree = (x, y, z = 0) => new THREE.Vector3(x, z, -y);

let robot = null;
new URDFLoader().load("/twin/urdf/alohamini_twin.urdf", (r) => {
  robot = r;
  robot.rotation.x = -Math.PI / 2;
  robot.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
  scene.add(robot);
}, undefined, (err) => toast(`Could not load the twin model: ${err}`));

// ---------------------------------------------------------------- state + UI
let cfg = { speeds: [{ xy: 0.15, theta: 45 }, { xy: 0.2, theta: 60 }, { xy: 0.25, theta: 75 }], lift_max_mm: 600 };
let speedIdx = 1;
let youControl = false;
let hasController = false;
let estop = false;
let lastState = null;
let liftGoal = null; // mm, null = hold
let liftDragging = false;
let liftKeyed = false;
const keys = new Set();

fetch("/config.json").then((r) => r.json()).then((c) => { cfg = c; renderSpeed(); }).catch(() => {});

function toast(text) {
  const t = $("toast");
  t.textContent = text;
  t.classList.add("show");
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => t.classList.remove("show"), 2600);
}

function renderSpeed() {
  document.querySelectorAll("#speed button").forEach((b) => b.classList.toggle("on", +b.dataset.i === speedIdx));
  const s = cfg.speeds[speedIdx];
  $("speed-hint").textContent = `${s.xy.toFixed(2)} m/s · ${s.theta}°/s`;
}
document.querySelectorAll("#speed button").forEach((b) =>
  b.addEventListener("click", () => { speedIdx = +b.dataset.i; renderSpeed(); }));

// Joint table rows are created once.
const jointCells = {};
for (const j of ARM_JOINTS) {
  const tr = document.createElement("tr");
  tr.innerHTML = `<td>${j.replace("_", " ")}</td><td data-side="left">—</td><td data-side="right">—</td>`;
  $("joints").appendChild(tr);
  jointCells[`left_${j}`] = tr.children[1];
  jointCells[`right_${j}`] = tr.children[2];
}
$("joints").insertAdjacentHTML("afterbegin", `<tr><td></td><td>L°</td><td>R°</td></tr>`);

// ---------------------------------------------------------------- input
const KEYMAP = { w: "w", a: "a", s: "s", d: "d", q: "q", e: "e", r: "r", f: "f",
  arrowup: "w", arrowdown: "s", arrowleft: "a", arrowright: "d" };

function setKey(k, down) {
  if (down) keys.add(k); else keys.delete(k);
  document.querySelectorAll(`kbd[data-key="${k}"]`).forEach((el) => el.classList.toggle("down", down));
}
function releaseAll() { [...keys].forEach((k) => setKey(k, false)); }

addEventListener("keydown", (ev) => {
  if (ev.target.closest?.("input, textarea")) return;
  if (ev.code === "Space") { ev.preventDefault(); if (!ev.repeat) setEstop(!estop); return; }
  if (ev.key >= "1" && ev.key <= "3") { speedIdx = +ev.key - 1; renderSpeed(); return; }
  const k = KEYMAP[ev.key.toLowerCase()];
  if (k) { ev.preventDefault(); setKey(k, true); }
});
addEventListener("keyup", (ev) => { const k = KEYMAP[ev.key.toLowerCase()]; if (k) setKey(k, false); });
addEventListener("blur", releaseAll);
document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });

document.querySelectorAll("kbd[data-key]").forEach((el) => {
  const k = el.dataset.key;
  el.addEventListener("pointerdown", (ev) => { el.setPointerCapture(ev.pointerId); setKey(k, true); });
  el.addEventListener("pointerup", () => setKey(k, false));
  el.addEventListener("pointercancel", () => setKey(k, false));
});

function setEstop(on) {
  estop = on;
  $("estop").classList.toggle("engaged", on);
  $("estop").firstChild.textContent = on ? "STOPPED" : "STOP";
  if (on) { releaseAll(); liftGoal = null; toast("Stopped. Press Space or click to resume."); }
}
$("estop").addEventListener("click", () => setEstop(!estop));

// Lift lever: drag sets an absolute goal; R/F nudges it.
const track = $("lever-track");
function goalFromPointer(ev) {
  const r = track.getBoundingClientRect();
  return Math.round(Math.min(Math.max((r.bottom - ev.clientY) / r.height, 0), 1) * cfg.lift_max_mm);
}
track.addEventListener("pointerdown", (ev) => {
  if (estop) return;
  liftDragging = true;
  track.setPointerCapture(ev.pointerId);
  liftGoal = goalFromPointer(ev);
});
track.addEventListener("pointermove", (ev) => { if (liftDragging) liftGoal = goalFromPointer(ev); });
track.addEventListener("pointerup", () => { liftDragging = false; });
$("lever-goal").addEventListener("keydown", (ev) => {
  const step = ev.shiftKey ? 50 : 10;
  const base = liftGoal ?? lastState?.lift_mm ?? 0;
  if (ev.key === "PageUp") liftGoal = Math.min(base + step, cfg.lift_max_mm);
  if (ev.key === "PageDown") liftGoal = Math.max(base - step, 0);
});

// ---------------------------------------------------------------- websocket
let ws = null;
function connect() {
  ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.onmessage = (ev) => onState(JSON.parse(ev.data));
  ws.onclose = () => {
    setPill("pill-link", "disconnected", "off");
    youControl = false;
    setTimeout(connect, 1000);
  };
}
connect();

$("btn-control").addEventListener("click", () => {
  if (ws?.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ type: youControl ? "release_control" : "take_control" }));
  if (youControl) releaseAll();
});

let lastSend = 0;
function sendInput(now) {
  if (!youControl || ws?.readyState !== WebSocket.OPEN || now - lastSend < 1000 / SEND_HZ) return;
  lastSend = now;
  const s = cfg.speeds[speedIdx];
  const axis = (pos, neg) => (keys.has(pos) ? 1 : 0) - (keys.has(neg) ? 1 : 0);
  const liftDir = axis("r", "f");
  const actual = lastState?.lift_mm ?? 0;
  // Hold-to-move like the keyboard client: the goal rides just ahead of the lift and is
  // dropped on release, so the lift stops where it is instead of coasting to a far goal.
  if (!estop && liftDir) {
    liftGoal = Math.min(Math.max(actual + liftDir * LIFT_KEY_LEAD, 0), cfg.lift_max_mm);
    liftKeyed = true;
  } else if (liftKeyed) {
    liftGoal = null;
    liftKeyed = false;
  }
  if (liftGoal !== null && !liftDragging && Math.abs(liftGoal - actual) <= LIFT_DONE_MM) liftGoal = null;
  const drive = { fwd: axis("w", "s"), left: axis("a", "d"), ccw: axis("q", "e") };
  const active = !estop && (drive.fwd || drive.left || drive.ccw || liftGoal !== null);
  ws.send(JSON.stringify({ type: "input", ...drive, speed: s.xy, turn: s.theta,
    lift_mm: estop ? null : liftGoal, active: Boolean(active), estop }));
}

function setPill(id, text, kind) {
  const el = $(id);
  el.textContent = text;
  el.className = `pill ${kind ? `pill-${kind}` : ""}`;
}

const fmt = (v, d = 2) => (v >= 0 ? " " : "") + v.toFixed(d);

function onState(st) {
  if (st.type !== "state") return;
  const prevControl = youControl;
  lastState = st;
  youControl = st.you_control;
  hasController = st.has_controller;
  if (prevControl && !youControl) { releaseAll(); liftGoal = null; }

  setPill("pill-link", st.online ? `online · ${st.link}` : `no feedback · ${st.link}`, st.online ? "ok" : "off");
  setPill("pill-mode", st.mode === "sim" ? "MuJoCo twin" : "REAL ROBOT", st.mode === "sim" ? "" : "warn");
  setPill("pill-control", youControl ? (st.commanding ? "you · driving" : "you · idle") : hasController ? "view only" : "no operator",
    youControl ? "ok" : "");
  const btn = $("btn-control");
  btn.textContent = youControl ? "Release control" : "Take control";
  btn.disabled = !youControl && hasController;

  const [x, y, yaw] = st.pose;
  $("t-pose").textContent = `${fmt(x)}, ${fmt(y)} m · ${Math.round((yaw * 180) / Math.PI)}°`;
  $("t-vel").textContent = `${fmt(st.vel[0])} ${fmt(st.vel[1])} m/s ${Math.round(st.vel[2])}°/s`;
  const c = st.cmd;
  $("t-cmd").textContent = st.commanding ? `${fmt(c["x.vel"])} ${fmt(c["y.vel"])} ${Math.round(c["theta.vel"])}°/s` : "quiet";
  $("t-rate").textContent = `${st.cmd_rate_hz} Hz · ${st.viewers} viewer${st.viewers === 1 ? "" : "s"}`;
  const sf = st.safety ?? {};
  $("t-safety").textContent = sf.watchdog_active ? "watchdog stop" : sf.owner ? `owner: ${sf.owner}` : sf.target_source ?? "—";

  $("lift-actual").textContent = `${st.lift_mm.toFixed(0)} mm`;
  $("lift-goal").textContent = liftGoal === null ? "hold" : `${liftGoal.toFixed(0)} mm`;
  const frac = (mm) => `${(Math.min(Math.max(mm / cfg.lift_max_mm, 0), 1) * 100).toFixed(2)}%`;
  $("lever-fill").style.height = frac(st.lift_mm);
  $("lever-actual").style.bottom = frac(st.lift_mm);
  const g = $("lever-goal");
  g.classList.toggle("show", liftGoal !== null);
  if (liftGoal !== null) g.style.bottom = frac(liftGoal);
  g.setAttribute("aria-valuenow", String(Math.round(liftGoal ?? st.lift_mm)));

  renderArms(st.arms ?? {});
  for (const [name, cell] of Object.entries(jointCells)) {
    const v = st.joints[name];
    cell.textContent = v === undefined ? "—" : Math.round((v * 180) / Math.PI);
  }

  if (robot) {
    const set = (n, v) => robot.joints[n]?.setJointValue(v);
    set("base_x", x);
    set("base_y", y);
    set("base_yaw", yaw);
    for (const [n, v] of Object.entries(st.joints)) {
      if (!n.startsWith("base_")) set(n, v);
    }
  }

  const p = toThree(x, y, 0.004);
  const last = trailN ? new THREE.Vector3().fromArray(trailPos, ((trailN - 1) % TRAIL_MAX) * 3) : null;
  if (!last || last.distanceTo(p) > 0.01) {
    p.toArray(trailPos, (trailN % TRAIL_MAX) * 3);
    trailN++;
    trailGeo.setDrawRange(0, Math.min(trailN, TRAIL_MAX));
    trailGeo.attributes.position.needsUpdate = true;
  }
}

const ARMS_TEXT = {
  leader: "The twin's arms follow the leader arms on this laptop.",
  robot: "Real follower arms, driven by leader-arm teleop. The web UI never commands them.",
  none: "Not connected. Start the server with --leader-left/--leader-right to mirror your leader arms.",
};
let armsKey = "";
function renderArms(arms) {
  const leaders = Object.entries(arms.leader ?? {});
  const key = JSON.stringify([arms.source, leaders.map(([s, v]) => [s, v.ok, v.error])]);
  if (key === armsKey) return; // avoid rebuilding the DOM 25x per second
  armsKey = key;
  $("arms-source").textContent = ARMS_TEXT[arms.source] ?? ARMS_TEXT.none;
  const box = $("leaders");
  const offline = leaders.filter(([, v]) => !v.ok);
  $("leaders-hint").textContent = offline.length
    ? `${offline[0][1].error ?? "offline"}. Retrying every second.`
    : "";
  box.replaceChildren(...leaders.map(([side, v]) => {
    const row = document.createElement("div");
    row.className = "leader";
    const name = document.createElement("b");
    name.textContent = `${side} · ${v.port}`;
    const state = document.createElement("span");
    state.className = v.ok ? "ok" : "bad";
    state.textContent = v.ok ? "reading" : "offline";
    if (!v.ok) row.title = v.error ?? "";
    row.append(name, state);
    return row;
  }));
}

// ---------------------------------------------------------------- follow camera + loop
let follow = true;
$("btn-follow").addEventListener("click", () => { follow = !follow; $("btn-follow").classList.toggle("on", follow); });
$("btn-reset").addEventListener("click", () => {
  const [x, y] = lastState?.pose ?? [0, 0];
  resetView();
  camera.position.add(toThree(x, y));
  controls.target.add(toThree(x, y));
});

const followTarget = new THREE.Vector3();
function frame(now) {
  sendInput(now);
  if (follow && lastState) {
    const [x, y] = lastState.pose;
    followTarget.copy(toThree(x, y, 0.55));
    const delta = followTarget.clone().sub(controls.target);
    controls.target.add(delta);
    camera.position.add(delta);
    sun.position.set(followTarget.x + 3, 6, followTarget.z + 2);
    sun.target.position.copy(followTarget);
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

addEventListener("resize", () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
