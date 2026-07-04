const state = {
  config: {},
  hasVideo: false,
  lastFrameTime: null,
};

const LABELS = {
  system: {
    system_idle: "系统空闲 / system_idle",
    task_running: "任务运行中 / task_running",
    system_error: "系统错误 / system_error",
    shutdown: "已关闭 / shutdown",
  },
  task: {
    tracking: "跟踪 / tracking",
    docking: "对接 / docking",
    charging: "充电 / charging",
    fish_control: "子机器人控制 / fish_control",
  },
  stage: {
    search: "搜索目标 / search",
    track: "视觉跟踪 / track",
    pre_align: "预对准 / pre_align",
    docked: "已对接 / docked",
    failed: "失败 / failed",
    running: "运行中 / running",
    completed: "已完成 / completed",
    stopped: "已停止 / stopped",
    idle: "空闲 / idle",
    lost: "目标丢失 / lost",
    predicted: "预测 / predicted",
    tracking: "跟踪 / tracking",
  },
  verticalMode: {
    visual_pid: "视觉 PID / visual_pid",
    hold_captured_ch3: "保持捕获 ch3 / hold_captured_ch3",
  },
  preAlignMode: {
    full_control: "全轴控制 / full_control",
    small_correction: "小修正 / small_correction",
    lock_horizontal: "锁定水平 / lock_horizontal",
  },
};

const rovButtons = Array.from(document.querySelectorAll("[data-rov]"));
const configForm = document.getElementById("configForm");

function $(id) {
  return document.getElementById(id);
}

async function requestJson(path, options = {}) {
  const response = await fetch(path, {
    headers: {"Content-Type": "application/json"},
    ...options,
  });
  const payload = await response.json();
  if (!response.ok || payload.ok === false) {
    throw new Error(payload.error || "request failed");
  }
  return payload;
}

function showToast(message) {
  const toast = $("toast");
  toast.textContent = message;
  toast.hidden = false;
  clearTimeout(showToast.timer);
  showToast.timer = setTimeout(() => {
    toast.hidden = true;
  }, 3600);
}

function handleUiError(error) {
  showToast(`操作失败 / Request failed: ${error.message}`);
}

function formatTime(epochSeconds) {
  if (!epochSeconds) return "-";
  return new Date(epochSeconds * 1000).toLocaleTimeString();
}

function formatEnum(value, labels) {
  if (value === undefined || value === null || value === "") return "-";
  return labels[value] || String(value);
}

function formatBool(value) {
  if (value === true) return "是 / true";
  if (value === false) return "否 / false";
  return "-";
}

function formatNumber(value, digits = 3, unit = "") {
  const number = Number(value);
  if (!Number.isFinite(number)) return "-";
  return `${number.toFixed(digits)}${unit}`;
}

function formatCompactObject(value) {
  if (!value || typeof value !== "object" || Array.isArray(value)) return "-";
  const entries = Object.entries(value).filter(([, item]) => item !== "" && item !== undefined && item !== null);
  if (entries.length === 0) return "-";
  return entries
    .slice(0, 8)
    .map(([key, item]) => `${key}:${formatObjectValue(item)}`)
    .join("  ");
}

function formatObjectValue(value) {
  if (typeof value === "number") {
    return Number.isInteger(value) ? String(value) : value.toFixed(3);
  }
  if (typeof value === "boolean") return value ? "true" : "false";
  return String(value);
}

function setText(id, value) {
  const element = $(id);
  if (element) element.textContent = value;
}

function setClass(id, className) {
  const element = $(id);
  if (element) element.className = className || "";
}

function taskStatusFrom(payload) {
  const taskStatus = payload?.rov?.last_status?.task_status || {};
  const currentTask = taskStatus.current_task || {};
  return {taskStatus, currentTask};
}

function renderStatus(payload) {
  const rov = payload.rov || {};
  const {taskStatus, currentTask} = taskStatusFrom(payload);
  const config = payload.config || {};
  state.config = config;
  renderConnection(payload, rov);
  renderSummary(taskStatus, currentTask, config);
  renderTaskStatus(currentTask, config, rov);
  renderVideoStatus(rov.video || {});
  renderPose(currentTask);
  fillConfigForm(config);
}

function renderConnection(payload, rov) {
  $("rovHost").value = rov.host || payload.defaults?.rov_host || $("rovHost").value;
  $("rovPort").value = rov.port || payload.defaults?.rov_port || $("rovPort").value;
  $("connectionLine").textContent = rov.connected
    ? `已连接 / Connected: ${rov.host}:${rov.port}`
    : `未连接 / Disconnected: ${payload.defaults?.rov_host || "127.0.0.1"}:${payload.defaults?.rov_port || 9002}`;
  $("connectionLine").className = rov.connected ? "connection-line ok" : "connection-line bad";
}

function renderSummary(taskStatus, currentTask, config) {
  setText("systemState", formatEnum(taskStatus.system_state, LABELS.system));
  setText("currentTask", formatEnum(currentTask.name, LABELS.task));
  setText("taskStage", formatEnum(currentTask.stage || currentTask.status, LABELS.stage));

  setText("preDockReady", formatBool(currentTask.pre_dock_ready));
  setClass("preDockReady", currentTask.pre_dock_ready === true ? "ok" : "");

  setText("motionEnabled", formatBool(config.enable_motion));
  setClass("motionEnabled", config.enable_motion === true ? "bad" : "ok");
}

function renderTaskStatus(currentTask, config, rov) {
  setText("missionMode", currentTask.mission_mode || "-");
  setText("trackingReady", formatBool(currentTask.tracking_ready));
  setClass("trackingReady", currentTask.tracking_ready === true ? "ok" : "");
  setText(
    "trackingVerticalMode",
    formatEnum(currentTask.tracking_vertical_mode || config.tracking_vertical_mode, LABELS.verticalMode),
  );
  setText(
    "preAlignMode",
    formatEnum(currentTask.pre_align_axis_mode || config.pre_align_axis_mode, LABELS.preAlignMode),
  );
  setText("capturedCh3", formatCapturedCh3(currentTask));
  setClass("capturedCh3", currentTask.captured_hold_ch3_available === true ? "ok" : "");
  setText("lastMessage", formatTime(rov.last_message_time));
}

function renderVideoStatus(video) {
  const hasFrame = video.has_frame === true;
  state.hasVideo = hasFrame;
  state.lastFrameTime = video.latest_frame_time || null;
  setText("videoState", hasFrame ? "有画面 / Live" : "无画面 / No frame");
  setClass("videoState", hasFrame ? "status-pill ok" : "status-pill");
  setText("videoFrameTime", formatTime(video.latest_frame_time));
  setText("videoFrameSize", video.latest_frame_size ? `${video.latest_frame_size} B` : "-");
  if (hasFrame) {
    showVideoFrame();
  } else {
    hideVideoFrame();
  }
}

function renderPose(currentTask) {
  const pose = currentTask.last_pose || {};
  const filtered = currentTask.filtered_state || {};
  const control = currentTask.control_cmd || {};
  const rcOverride = currentTask.rc_override || {};

  const detected = pose.detected;
  const poseValid = pose.pose_valid !== undefined ? pose.pose_valid : detected;
  setText("poseDetected", formatBool(detected));
  setClass("poseDetected", detected === true ? "ok" : "");
  setText("poseValid", formatBool(poseValid));
  setClass("poseValid", poseValid === true ? "ok" : "bad");
  setText("poseReject", pose.reject_reason || "-");
  setText("poseReprojection", formatNumber(pose.reprojection_error_px, 3, " px"));
  setText("poseX", formatNumber(pose.x, 3));
  setText("poseY", formatNumber(pose.y, 3));
  setText("poseZ", formatNumber(pose.z, 3));
  setText("poseYaw", formatNumber(pose.yaw, 2));

  setText("bodyForward", formatNumber(filtered.forward_m, 3));
  setText("bodyRight", formatNumber(filtered.right_m, 3));
  setText("bodyUp", formatNumber(filtered.up_m, 3));
  setText("yawError", formatNumber(filtered.yaw_error_deg, 2));
  setText("filteredZ", formatNumber(filtered.z, 3));
  setText("filteredYaw", formatNumber(filtered.yaw, 2));

  setText("controlCommand", formatCompactObject(control));
  setText("rcOverride", formatCompactObject(rcOverride));
}

function formatCapturedCh3(task) {
  if (!task || task.captured_hold_ch3_available !== true) return "未捕获 / not captured";
  const pwm = task.captured_hold_ch3_pwm;
  const capturedAt = formatTime(task.captured_hold_ch3_time);
  return `${pwm} PWM (${capturedAt})`;
}

function fillConfigForm(config) {
  if (!configForm) return;
  for (const element of Array.from(configForm.elements)) {
    if (!element.name || !(element.name in config)) continue;
    if (element.type === "checkbox") {
      element.checked = Boolean(config[element.name]);
    } else if (document.activeElement !== element) {
      element.value = config[element.name];
    }
  }
}

async function refreshStatus() {
  try {
    const payload = await requestJson("/api/status");
    renderStatus(payload);
  } catch (error) {
    handleUiError(error);
  }
}

function configureVideoStream() {
  const frame = $("videoFrame");
  if (!frame) return;
  frame.onload = showVideoFrame;
  frame.onerror = () => {
    hideVideoFrame();
    setTimeout(() => {
      frame.src = `/api/video.mjpg?fps=25&t=${Date.now()}`;
    }, 1000);
  };
}

function showVideoFrame() {
  const frame = $("videoFrame");
  const placeholder = $("videoPlaceholder");
  if (!frame || !placeholder) return;
  frame.hidden = false;
  placeholder.hidden = true;
}

function hideVideoFrame() {
  const frame = $("videoFrame");
  const placeholder = $("videoPlaceholder");
  if (!frame || !placeholder) return;
  frame.hidden = true;
  placeholder.hidden = false;
}

async function connectRov() {
  const host = $("rovHost").value.trim() || "127.0.0.1";
  const port = Number($("rovPort").value || 9002);
  const payload = await requestJson("/api/connect", {
    method: "POST",
    body: JSON.stringify({host, port}),
  });
  renderStatus({...payload, config: state.config, defaults: {rov_host: host, rov_port: port}});
  showToast("连接成功 / Connected");
}

async function disconnectRov() {
  await requestJson("/api/disconnect", {method: "POST", body: "{}"});
  await refreshStatus();
  showToast("已断开连接 / Disconnected");
}

async function sendRovCommand(command) {
  await requestJson("/api/command", {
    method: "POST",
    body: JSON.stringify({rov: command}),
  });
  showToast(`已发送 / Sent: ${command}`);
  await refreshStatus();
}

function collectConfigUpdates() {
  const updates = {};
  for (const element of Array.from(configForm.elements)) {
    if (!element.name) continue;
    if (element.type === "checkbox") {
      updates[element.name] = element.checked;
    } else if (element.dataset.valueType === "string") {
      updates[element.name] = element.value;
    } else if (element.value !== "") {
      updates[element.name] = Number(element.value);
    }
  }
  return updates;
}

async function saveConfig() {
  const updates = collectConfigUpdates();
  const confirmMotion = $("confirmMotion").checked;
  const payload = await requestJson("/api/config", {
    method: "POST",
    body: JSON.stringify({updates, confirm_motion: confirmMotion}),
  });
  state.config = payload.config;
  fillConfigForm(payload.config);
  showToast(
    payload.restart_required
      ? "配置已保存，重启 main.py 后生效 / Config saved. Restart main.py to apply."
      : "配置已保存 / Config saved",
  );
  await refreshStatus();
}

$("connectBtn").addEventListener("click", () => connectRov().catch(handleUiError));
$("disconnectBtn").addEventListener("click", () => disconnectRov().catch(handleUiError));
$("refreshBtn").addEventListener("click", refreshStatus);
$("reloadConfigBtn").addEventListener("click", refreshStatus);
$("saveConfigBtn").addEventListener("click", () => saveConfig().catch(handleUiError));

for (const button of rovButtons) {
  button.addEventListener("click", () => sendRovCommand(button.dataset.rov).catch(handleUiError));
}

configureVideoStream();
refreshStatus();
setInterval(refreshStatus, 1000);
