// Operator panel: talks to tewatashi_task through rosbridge (roslibjs).
// Only Trigger/SetBool services and one status topic are used.
"use strict";

const host = window.location.hostname || "127.0.0.1";
const params = new URLSearchParams(window.location.search);
const ROSBRIDGE_URL = params.get("rosbridge") || `ws://${host}:9090`;
const VIEWER_URL = params.get("viewer") || `http://${host}:5000`;
const NODE = "/tewatashi_task";
const VIDEO_URL = params.get("video") || `http://${host}:8081`;
const CAMERA_TOPIC = params.get("camera") || "/camera/camera/color/image_raw";

const $ = (id) => document.getElementById(id);
$("viewer").src = VIEWER_URL;
// web_video_server keeps the MJPEG stream open and shows frames as they arrive.
$("img-camera").src = `${VIDEO_URL}/stream?topic=${CAMERA_TOPIC}&width=640`;
$("img-sam3").src = `${VIDEO_URL}/stream?topic=/sam3_server/debug_image&width=640`;

let ros = null;
let connected = false;
let lastStatus = null;

function connect() {
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
  ros.on("connection", () => {
    connected = true;
    $("conn").textContent = "接続中";
    $("conn").className = "badge on";
    subscribeStatus();
    subscribeJoints();
    render();
  });
  ros.on("close", () => {
    connected = false;
    $("conn").textContent = "未接続（再接続を試みています）";
    $("conn").className = "badge off";
    render();
    setTimeout(connect, 2000);
  });
  ros.on("error", () => {});
}

function subscribeStatus() {
  const topic = new ROSLIB.Topic({
    ros,
    name: `${NODE}/status`,
    messageType: "std_msgs/msg/String",
  });
  topic.subscribe((msg) => {
    try {
      lastStatus = JSON.parse(msg.data);
    } catch (e) {
      return;
    }
    render();
  });
}

// Same order as task.yaml / motion.py (UR_JOINTS). /joint_states comes in a
// different order, so values are always matched by name.
const UR_JOINTS = [
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint",
];
let jointValues = null;

function subscribeJoints() {
  const topic = new ROSLIB.Topic({
    ros,
    name: "/joint_states",
    messageType: "sensor_msgs/msg/JointState",
    throttle_rate: 200,
  });
  topic.subscribe((msg) => {
    const byName = {};
    msg.name.forEach((n, i) => (byName[n] = msg.position[i]));
    if (UR_JOINTS.every((n) => n in byName)) {
      jointValues = UR_JOINTS.map((n) => byName[n]);
      renderJoints();
    }
  });
}

function renderJoints() {
  const table = $("joints");
  table.innerHTML = "";
  UR_JOINTS.forEach((n, i) => {
    const tr = document.createElement("tr");
    const rad = jointValues ? jointValues[i] : NaN;
    const deg = (rad * 180) / Math.PI;
    tr.innerHTML =
      `<td>${n.replace("_joint", "")}</td>` +
      `<td class="num">${jointValues ? deg.toFixed(1) + "°" : "-"}</td>` +
      `<td class="num muted">${jointValues ? rad.toFixed(4) : "-"}</td>`;
    table.appendChild(tr);
  });
  $("btn-copy").disabled = !jointValues;
}

$("btn-copy").onclick = async () => {
  if (!jointValues) return;
  const text = "[" + jointValues.map((v) => v.toFixed(4)).join(", ") + "]";
  try {
    await navigator.clipboard.writeText(text);
    $("copy-msg").textContent = "コピーしました";
  } catch (e) {
    $("copy-msg").textContent = text; // fallback: show it to copy by hand
  }
  setTimeout(() => ($("copy-msg").textContent = ""), 4000);
};

function call(name, type, request, onDone) {
  if (!connected) return;
  const srv = new ROSLIB.Service({ ros, name: `${NODE}/${name}`, serviceType: type });
  srv.callService(
    new ROSLIB.ServiceRequest(request),
    (res) => onDone && onDone(res),
    (err) => flash(`呼び出し失敗: ${name} (${err})`)
  );
}

function trigger(name) {
  call(name, "std_srvs/srv/Trigger", {}, (res) => {
    if (!res.success) flash(res.message);
  });
}

function flash(text) {
  const li = document.createElement("li");
  li.className = "error";
  li.textContent = text;
  $("log").prepend(li);
}

function render() {
  const s = lastStatus;
  const ok = connected && s !== null;
  const busy = ok && s.busy;

  $("btn-stop").disabled = !connected;
  $("btn-start").disabled = !ok || busy;
  $("btn-home").disabled = !ok || busy;
  $("chk-approval").disabled = !ok || s.mode === "real";

  if (!ok) return;

  $("mode").textContent = s.mode === "real" ? "実機" : "模擬ハードウェア";
  $("mode").className = "badge " + (s.mode === "real" ? "real" : "mock");
  $("chk-approval").checked = s.require_approval;
  $("task").textContent = s.task || "-";
  $("state").textContent = s.state || "-";
  $("result").textContent = s.last_result || "-";

  $("approval").classList.toggle("hidden", !s.waiting_approval);
  $("approval-text").textContent = s.approval_text;

  const log = $("log");
  log.innerHTML = "";
  for (const line of [...s.log].reverse()) {
    const li = document.createElement("li");
    if (line.error) li.className = "error";
    li.textContent = `${line.t}  ${line.text}`;
    log.appendChild(li);
  }
}

$("btn-stop").onclick = () => trigger("stop");
$("btn-start").onclick = () => trigger("start");
$("btn-home").onclick = () => trigger("home");
$("btn-approve").onclick = () => trigger("approve");
$("chk-approval").onchange = (e) =>
  call("set_require_approval", "std_srvs/srv/SetBool", { data: e.target.checked }, (res) => {
    if (!res.success) flash(res.message);
  });

document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") trigger("stop");
});

renderJoints();
render();
connect();
