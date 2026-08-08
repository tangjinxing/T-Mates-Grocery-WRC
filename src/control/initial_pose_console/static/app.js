const axes = ["x","y","z","rx","ry","rz"];
let latest = null;
let activeCamera = null;
let busy = false;
let registrySignature = "";
let scopeDirty = false;

document.querySelectorAll(".arm").forEach(card => {
  const side = card.dataset.side;
  const grid = card.querySelector(".pose-grid");
  grid.innerHTML = axes.map(a => `<label>${a}<input data-pose="${side}" data-axis="${a}" type="number" step="0.0001"></label>`).join("");
});

function log(message, kind="") {
  const node = document.querySelector("#log");
  const stamp = new Date().toLocaleTimeString();
  node.textContent = `[${stamp}] ${message}\n` + node.textContent;
  if (kind === "error") console.error(message);
}

async function api(url, options={}) {
  const response = await fetch(url, {headers:{"Content-Type":"application/json"}, ...options});
  let body = {};
  try { body = await response.json(); } catch (_) {}
  if (!response.ok) throw new Error(body.detail || `HTTP ${response.status}`);
  return body.data;
}

async function act(label, fn) {
  if (busy) return log("已有界面操作正在执行", "error");
  busy = true;
  try { const data = await fn(); log(`${label}：成功`); return data; }
  catch (error) { log(`${label}：${error.message}`, "error"); alert(error.message); }
  finally { busy = false; }
}

function poseInputs(side) {
  return [...document.querySelectorAll(`[data-pose="${side}"]`)].map(input => Number(input.value));
}

function fillPose(side) {
  const pose = latest?.robot?.arms?.[side]?.state?.pose_base_to_gripper_m_rad;
  if (!pose) return alert(`${side}臂没有实际位姿，请先连接`);
  document.querySelectorAll(`[data-pose="${side}"]`).forEach((input, i) => input.value = Number(pose[i]).toFixed(6));
}

document.querySelectorAll("[data-camera]").forEach(button => button.onclick = () => act("切换相机", async () => {
  const name = button.dataset.camera;
  await api(`/api/camera/${name}/activate`, {method:"POST"});
  activeCamera = name;
  document.querySelectorAll("[data-camera]").forEach(b => b.classList.toggle("active", b.dataset.camera === name));
  const image = document.querySelector("#video");
  image.src = `/api/camera/video?t=${Date.now()}`;
  image.style.display = "block";
  document.querySelector("#screen-hint").style.display = "none";
}));

document.querySelector("#camera-stop").onclick = () => act("释放相机", async () => {
  await api("/api/camera/stop", {method:"POST"}); activeCamera = null;
  const image = document.querySelector("#video"); image.src=""; image.style.display="none";
  document.querySelector("#screen-hint").style.display="block";
  document.querySelectorAll("[data-camera]").forEach(b => b.classList.remove("active"));
});
document.querySelector("#snapshot").onclick = () => act("保存快照", () => api("/api/camera/snapshot", {method:"POST"}));

document.querySelectorAll("[data-connect]").forEach(button => button.onclick = () => act("连接机械臂", () => api(`/api/arm/${button.dataset.connect}/connect`, {method:"POST"})));
document.querySelectorAll("[data-disconnect]").forEach(button => button.onclick = () => act("断开机械臂", () => api(`/api/arm/${button.dataset.disconnect}/disconnect`, {method:"POST"})));
document.querySelector("#head-connect").onclick = () => act("连接头部", () => api("/api/head/connect", {method:"POST"}));
document.querySelector("#head-disconnect").onclick = () => act("断开头部", () => api("/api/head/disconnect", {method:"POST"}));

document.querySelectorAll("[data-fill]").forEach(button => button.onclick = () => fillPose(button.dataset.fill));
document.querySelectorAll("[data-move]").forEach(button => button.onclick = async () => {
  const side = button.dataset.move, pose = poseInputs(side);
  if (pose.some(v => !Number.isFinite(v))) return alert("请填写完整的 6D 位姿");
  if (!confirm(`确认以低速移动${side === "left" ? "左" : "右"}臂？\n目标 [${pose.join(", ")}]`)) return;
  const speed = Number(document.querySelector(`[data-speed="${side}"]`).value);
  await act("机械臂运动", () => api(`/api/arm/${side}/move`, {method:"POST", body:JSON.stringify({pose,speed,confirmed:true})}));
});

document.querySelector("#head-fill").onclick = () => {
  const head = latest?.robot?.head;
  if (head?.yaw == null || head?.pitch == null) return alert("没有头部实际读数");
  document.querySelector("#head-yaw").value=head.yaw; document.querySelector("#head-pitch").value=head.pitch;
};
document.querySelector("#head-move").onclick = async () => {
  const yaw=Number(document.querySelector("#head-yaw").value), pitch=Number(document.querySelector("#head-pitch").value);
  if (!confirm(`确认移动头部到 yaw=${yaw}, pitch=${pitch}？`)) return;
  await act("头部运动", () => api("/api/head/move", {method:"POST",body:JSON.stringify({yaw,pitch,confirmed:true})}));
};

document.querySelector("#lift-fill").onclick = () => {
  const height = latest?.robot?.lift?.state?.height;
  if (height == null) return alert("没有升降柱实际高度，请先连接左臂");
  document.querySelector("#lift-height").value = height;
};
document.querySelector("#lift-move").onclick = async () => {
  const height = Number(document.querySelector("#lift-height").value);
  const speed = Number(document.querySelector("#lift-speed").value);
  if (!Number.isInteger(height) || height < 100 || height > 1350) return alert("目标高度必须是100～1350 mm的整数");
  if (!Number.isInteger(speed) || speed < 1 || speed > 20) return alert("速度必须是1～20%的整数");
  if (!confirm(`确认移动升降柱到 ${height} mm，速度 ${speed}%？\n请确保躯干和双臂周围无障碍物。`)) return;
  await act("升降柱运动", () => api("/api/lift/move", {method:"POST", body:JSON.stringify({height,speed,confirmed:true})}));
};

function selectedRegistryName() {
  return document.querySelector("#registry-select").value;
}

function registryComponentSummary(preset) {
  if (!preset) return "未找到预设";
  const labels = {head:"头部",torso:"躯干",left_arm:"左臂",right_arm:"右臂"};
  const applied = preset.apply_components || [];
  const available = {
    head: preset.head?.yaw != null && preset.head?.pitch != null,
    torso: preset.torso?.height != null,
    left_arm: Boolean(preset.left_arm?.pose_6d),
    right_arm: Boolean(preset.right_arm?.pose_6d),
  };
  const missing = applied.filter(component => !available[component]);
  return `状态=${preset.status} · 执行=${applied.map(x=>labels[x]).join("、") || "未设置"} · 缺少=${missing.map(x=>labels[x]).join("、") || "无"}`;
}

function renderRegistry(forceScope=false) {
  const presets = latest?.pose_registry?.presets || {};
  const names = Object.keys(presets);
  const select = document.querySelector("#registry-select");
  const previous = select.value;
  const signature = names.join("|");
  if (signature !== registrySignature) {
    select.innerHTML = names.map(name => `<option value="${name}">${name} — ${presets[name].description || ""}</option>`).join("");
    registrySignature = signature;
    if (names.includes(previous)) select.value = previous;
  }
  const preset = presets[select.value];
  document.querySelector("#registry-status").textContent = registryComponentSummary(preset);
  document.querySelector("#registry-preview").textContent = preset ? JSON.stringify(preset, null, 2) : "";
  if (preset && (forceScope || !scopeDirty)) {
    const applied = new Set(preset.apply_components || []);
    document.querySelectorAll("[data-apply-component]").forEach(input => input.checked = applied.has(input.dataset.applyComponent));
  }
}

document.querySelector("#registry-select").onchange = () => { scopeDirty=false; renderRegistry(true); };
document.querySelectorAll("[data-apply-component]").forEach(input => input.onchange = () => { scopeDirty=true; });
document.querySelector("#registry-save-scope").onclick = async () => {
  const name = selectedRegistryName();
  const components = [...document.querySelectorAll("[data-apply-component]:checked")].map(input => input.dataset.applyComponent);
  if (!name) return alert("请先选择预设名称");
  if (!components.length) return alert("至少选择一个参与执行的部件");
  if (!confirm(`确认更新 ${name} 的执行范围为：${components.join(", ")}？`)) return;
  const data = await act("保存执行范围", () => api(`/api/pose-registry/${name}/apply-components`, {method:"PUT", body:JSON.stringify({components})}));
  if (data) { scopeDirty=false; await refresh(); renderRegistry(true); }
};
document.querySelectorAll("[data-capture-component]").forEach(button => button.onclick = async () => {
  const name = selectedRegistryName();
  const component = button.dataset.captureComponent;
  if (!name) return alert("请先选择预设名称");
  if (!confirm(`确认用硬件当前实际读数更新 ${name} 的 ${component}？\n旧YAML会自动备份。`)) return;
  const data = await act("更新统一预设", () => api(`/api/pose-registry/${name}/capture/${component}`, {method:"POST"}));
  if (data) await refresh();
});
document.querySelector("#registry-view").onclick = () => {
  const preset = latest?.pose_registry?.presets?.[selectedRegistryName()];
  if (!preset) return alert("未找到当前预设");
  document.querySelector("#json-content").textContent = JSON.stringify(preset, null, 2);
  document.querySelector("#json-dialog").showModal();
};
document.querySelector("#registry-create").onclick = async () => {
  const name = document.querySelector("#registry-new-name").value.trim();
  const description = document.querySelector("#registry-new-description").value.trim();
  if (!name || !description) return alert("请填写英文ID和中文说明");
  if (!confirm(`确认创建空预设 ${name}？`)) return;
  const data = await act("创建预设", () => api("/api/pose-registry", {method:"POST", body:JSON.stringify({name,description})}));
  if (data) {
    registrySignature = "";
    await refresh();
    document.querySelector("#registry-select").value = name;
    renderRegistry();
  }
};

document.querySelectorAll("[data-save]").forEach(button => button.onclick = async () => {
  const name=button.dataset.save;
  if (!confirm("确认用硬件当前实际读数覆盖该初始拍照姿态？旧 JSON 会自动备份。")) return;
  await act("保存实际姿态", () => api(`/api/preset/${name}/save`, {method:"POST"}));
});
document.querySelectorAll("[data-load]").forEach(button => button.onclick = async () => {
  const data = await act("读取预设", () => api(`/api/preset/${button.dataset.load}`));
  if (data) { document.querySelector("#json-content").textContent=JSON.stringify(data,null,2); document.querySelector("#json-dialog").showModal(); }
});
document.querySelectorAll("[data-preset-move]").forEach(button => button.onclick = async () => {
  const name=button.dataset.presetMove;
  if (!confirm("确认移动到已保存的实际姿态？请确保运动空间无人员和障碍物。")) return;
  await act("移动到保存姿态", () => api(`/api/preset/${name}/move`, {method:"POST",body:JSON.stringify({confirmed:true})}));
});
document.querySelector("#stop-all").onclick = () => act("全局停止", () => api("/api/motion/stop", {method:"POST"}));

function armText(side) {
  const arm=latest?.robot?.arms?.[side]; if (!arm?.connected) return "尚未连接";
  const pose=arm.state?.pose_base_to_gripper_m_rad || [];
  const joints=arm.state?.joints_deg || [];
  return `6D [${pose.map(v=>Number(v).toFixed(4)).join(", ")}]\n关节 [${joints.map(v=>Number(v).toFixed(2)).join(", ")}]${arm.error ? `\n错误: ${arm.error}`:""}`;
}

async function refresh() {
  try {
    latest=await api("/api/status");
    document.querySelector("#left-live").textContent=armText("left");
    document.querySelector("#right-live").textContent=armText("right");
    const head=latest.robot.head;
    document.querySelector("#head-live").textContent=head.connected ? `实际 yaw=${head.yaw ?? "-"}, pitch=${head.pitch ?? "-"}${head.error?` · ${head.error}`:""}` : "尚未连接";
    const lift=latest.robot.lift;
    document.querySelector("#lift-live").textContent=lift.state ? `实际高度=${lift.state.height}, 电流=${lift.state.current}, 错误=${lift.state.err}, 模式=${lift.state.mode}` : `尚无升降柱状态${lift.error?` · ${lift.error}`:""}`;
    const cam=latest.camera;
    const availability=Object.entries(cam.configured).map(([k,v])=>`${v.label}:${v.present?"在线":"未发现"}`).join(" · ");
    document.querySelector("#camera-status").textContent=`${availability} | 当前=${cam.active||"无"} | ${cam.fps||0} FPS | 最新帧 ${cam.latest_frame_age_ms??"-"} ms${cam.error?` | ${cam.error}`:""}`;
    document.querySelector("#hardware-summary").textContent=`左臂 ${latest.robot.arms.left.connected?"已连接":"未连接"} / 右臂 ${latest.robot.arms.right.connected?"已连接":"未连接"} / 头部 ${head.connected?"已连接":"未连接"}${latest.robot.motion_busy?"\n运动任务执行中":""}`;
    renderRegistry();
  } catch(error) { log(`状态刷新失败：${error.message}`, "error"); }
}
refresh(); setInterval(refresh, 700);
