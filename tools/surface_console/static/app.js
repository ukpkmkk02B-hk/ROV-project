const state = {
  config: {},
};

const LABELS = {
  system: {
    system_idle: "系统空闲 / system_idle",
    task_running: "任务运行中 / task_running",
    system_error: "系统错误 / system_error",
    shutdown: "已关闭 / shutdown",
  },
  task: {
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

  $("rovHost").value = rov.host || payload.defaults?.rov_host || $("rovHost").value;
  $("rovPort").value = rov.port || payload.defaults?.rov_port || $("rovPort").value;
  $("connectionLine").textContent = rov.connected
    ? `已连接 / Connected: ${rov.host}:${rov.port}`
    : `未连接 / Disconnected: ${payload.defaults?.rov_host || "127.0.0.1"}:${payload.defaults?.rov_port || 9002}`;
  $("connectionLine").className = rov.connected ? "connection-line ok" : "connection-line bad";

  $("systemState").textContent = formatEnum(taskStatus.system_state, LABELS.system);
  $("currentTask").textContent = formatEnum(currentTask.name, LABELS.task);
  $("taskStage").textContent = formatEnum(currentTask.stage || currentTask.status, LABELS.stage);

  $("preDockReady").textContent = formatBool(currentTask.pre_dock_ready);
  $("preDockReady").className = currentTask.pre_dock_ready === true ? "ok" : "";

  $("motionEnabled").textContent = formatBool(config.enable_motion);
  $("motionEnabled").className = config.enable_motion === true ? "bad" : "ok";

  $("trackingVerticalMode").textContent = formatEnum(
    currentTask.tracking_vertical_mode || config.tracking_vertical_mode,
    LABELS.verticalMode,
  );
  $("preAlignMode").textContent = formatEnum(
    currentTask.pre_align_axis_mode || config.pre_align_axis_mode,
    LABELS.preAlignMode,
  );
  $("capturedCh3").textContent = formatCapturedCh3(currentTask);
  $("capturedCh3").className = currentTask.captured_hold_ch3_available === true ? "ok" : "";
  $("lastMessage").textContent = formatTime(rov.last_message_time);

  fillConfigForm(config);
}

function formatCapturedCh3(task) {
  if (!task || task.captured_hold_ch3_available !== true) return "未捕获 / not captured";
  const pwm = task.captured_hold_ch3_pwm;
  const capturedAt = formatTime(task.captured_hold_ch3_time);
  return `${pwm} PWM (${capturedAt})`;
}

function fillConfigForm(config) {
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
  showToast(payload.restart_required ? "配置已保存，重启 main.py 后生效 / Config saved. Restart main.py to apply." : "配置已保存 / Config saved");
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

refreshStatus();
setInterval(refreshStatus, 1000);
