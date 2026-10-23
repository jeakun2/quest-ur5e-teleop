// WebXR client: streams controller poses/buttons to the bridge over WebSocket, renders the
// bridge's status / menu / flow text in a DOM overlay, and draws a translucent UR5e
// "hologram" (real joint angles from the bridge) plus the TCP / hand paths with three.js.
// Frame: WebXR 'local-floor' reference space (right-handed, +Y up, -Z forward, metres).
import * as THREE from 'three';
import { UR5eGhost, PathLine } from './ghost.js';
import { XRMenu, menuGroupOf, PILL_ON, PILL_OFF } from './xrmenu.js';
import { XRCams } from './xrcams.js';

const logQueue = [];
const log = (m) => {
  const el = document.getElementById('log'); el.textContent = (m + '\n' + el.textContent).slice(0, 4000);
  const pkt = { type: 'log', msg: String(m), ua: navigator.userAgent.slice(0, 80) };
  try { if (window.__wsSend && window.__wsSend(pkt)) return; } catch {}
  logQueue.push(pkt); if (logQueue.length > 50) logQueue.shift();
};
const $ = (id) => document.getElementById(id);
const hud = $('hud');
const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
window.addEventListener('error', (e) => log('JS error: ' + e.message));

// ---------------- WebSocket ----------------
let ws = null, wsOpen = false, sent = 0, lastRateT = performance.now(), rtt = null;
window.__wsSend = (o) => { if (wsOpen && ws) { ws.send(JSON.stringify(o)); return true; } return false; };
function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => {
    wsOpen = true; $('ws-state').textContent = 'WebSocket: connected';
    while (logQueue.length) { try { ws.send(JSON.stringify(logQueue.shift())); } catch { break; } }
    log('ws connected');
  };
  ws.onclose = () => { wsOpen = false; $('ws-state').textContent = 'WebSocket: closed, retrying…'; setTimeout(connect, 1000); };
  ws.onerror = () => { $('ws-state').textContent = 'WebSocket: error'; };
  ws.onmessage = (ev) => {
    let d; try { d = JSON.parse(ev.data); } catch { return; }
    if (d.type === 'pong') { rtt = performance.now() - d.t; return; }
    if (d.type === 'status') { renderStatus(d); updateScene(d); }
  };
}
connect();
setInterval(() => { if (wsOpen) ws.send(JSON.stringify({ type: 'ping', t: performance.now() })); }, 1000);
setInterval(() => {
  const now = performance.now(); const hz = sent * 1000 / (now - lastRateT); sent = 0; lastRateT = now;
  $('hud-rate').textContent = hz.toFixed(0) + ' Hz';
  $('hud-rtt').textContent = rtt == null ? '– ms' : rtt.toFixed(0) + ' ms';
}, 1000);
const send = (obj) => { if (wsOpen && ws.bufferedAmount < 4096) { ws.send(JSON.stringify(obj)); sent++; } };

// ---------------- HUD rendering ----------------
function renderStatus(d) {
  const st = $('hud-state'); st.textContent = d.state;
  st.className = 'state ' + (d.state.includes('ENGAGED') ? 'engaged' : d.state.startsWith('UNSAFE') ? 'bad' : 'warn');
  $('hud-tcp').textContent = d.tcp ? d.tcp.slice(0, 3).map(v => v.toFixed(3)).join(' ') : '–';
  $('hud-rot').textContent = d.rot ? 'on' : 'off';
  const ui = d.ui || {};
  $('hud-mode').textContent = ui.mode || '–';
  $('hud-scale').textContent = ui.scale != null ? 'x' + ui.scale : '–';
  const chips = [];
  if (ui.locks) for (const k of ['x', 'y', 'z', 'rot']) if (ui.locks[k]) chips.push(`<span class="on">${k.toUpperCase()} 고정</span>`);
  if (ui.map_mode === 'absolute') chips.push('<span class="on">절대</span>');
  if (ui.recording) chips.push(`<span class="rec">● REC ${ui.recording.seconds}s</span>`);
  if (ui.playing) chips.push(`<span class="on">▶ ${esc(ui.playing.path)} ${(ui.playing.progress * 100).toFixed(0)}%</span>`);
  if (ui.ghost) chips.push(`<span class="ghost">홀로그램${d.q ? '' : ' (TCP만)'}</span>`);
  if (ui.view && ui.view !== 'default') chips.push(`<span>Isaac 시점 ${esc(ui.view)}</span>`);
  updateCams(ui.cams || null);           // DOM figures (2D test / dom-overlay, hidden by #hud.xr in a session)
  xrCams.setCams(ui.cams || null);       // world-space windows by the right hand (headset)
  if (d.gripper != null) chips.push(`<span>그리퍼 ${(d.gripper * 100).toFixed(0)}%</span>`);
  $('hud-chips').innerHTML = chips.join('');
  $('hud-msg').textContent = ui.msg || '';
  const menu = $('hud-menu');
  if (ui.menu) {
    menu.hidden = false;
    menu.innerHTML = renderMenu(ui.menu.items);
    // the item list scrolls when the overlay is shorter than the menu; keep the cursor row visible
    const sel = menu.querySelector('.item.sel');
    if (sel && sel.scrollIntoView) sel.scrollIntoView({ block: 'nearest' });
    // in the headset the menu is a 3D panel next to the right controller (the DOM copy is hidden
    // by #hud.xr while a session runs); in the 2D test both are shown
    xrMenu.setItems(ui.menu.items, ui.menu.title); xrMenu.show();
  } else { menu.hidden = true; xrMenu.hide(); }
  const flow = $('hud-flow');
  if (ui.flow) {
    flow.hidden = false;
    flow.innerHTML = `<div class="title">${esc(ui.flow.title)}</div>` +
      ui.flow.lines.map(l => `<div class="line">${esc(l)}</div>`).join('');
  } else flow.hidden = true;
}

// DOM options menu (dom-overlay / 2D test). The bridge owns the item list, order and cursor; the
// page only groups the rows by key prefix (grouping shared with the 3D panel: MENU_GROUPS /
// menuGroupOf in xrmenu.js) and renders on/off values as pills.
function renderMenu(items) {
  let html = '<div class="list">', grp = null;
  for (const it of items) {
    const g = menuGroupOf(it.key);
    if (g !== grp) { grp = g; if (g) html += `<div class="grp">${esc(g)}</div>`; }
    const v = it.value == null ? '' : String(it.value);
    const pill = PILL_ON.includes(v) ? ' pill on' : PILL_OFF.includes(v) ? ' pill off' : '';
    const cls = 'item' + (it.sel ? ' sel' : '') + (it.key === 'close' ? ' close' : '');
    html += `<div class="${cls}" data-key="${esc(it.key)}"><span>${esc(it.label)}</span>` +
            (v ? `<span class="val${pill}">${esc(v)}</span>` : '') + `</div>`;
  }
  html += '</div><div class="hint">스틱 <b>▲▼</b> 이동 · <b>A</b> 선택 · <b>B</b> 닫기</div>';
  return html;
}
window.__renderStatus = renderStatus;   // test hook (Playwright / console): inject a fake status message

// ---------------- Isaac overlay cameras (2D images in the HUD) ----------------
const camState = { on: {}, timers: {} };
// a missing frame (404 / stream not up yet) shows a dim "no frame" placeholder instead of a broken image
for (const fig of $('hud-cams').querySelectorAll('figure')) {
  const img = fig.querySelector('img');
  img.onerror = () => fig.classList.add('nf');
  img.onload = () => fig.classList.remove('nf');
}
function updateCams(cams) {
  const box = $('hud-cams');
  if (!cams) { box.hidden = true; return; }
  let any = false;
  for (const fig of box.querySelectorAll('figure')) {
    const name = fig.dataset.cam, on = !!cams[name];
    fig.hidden = !on; any = any || on;
    if (on && !camState.timers[name]) {
      const img = fig.querySelector('img');
      fig.classList.add('nf');   // placeholder until the first frame arrives
      camState.timers[name] = setInterval(() => { img.src = `cam/${name}.jpg?t=${Date.now()}`; }, 125);
    } else if (!on && camState.timers[name]) {
      clearInterval(camState.timers[name]); delete camState.timers[name];
    }
  }
  box.hidden = !any || hud.hidden;
}

// ---------------- three.js scene: ghost robot + paths ----------------
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor(0x000000, 0);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
$('view').appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, 16 / 9, 0.02, 50);
scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.0); sun.position.set(1, 3, 2); scene.add(sun);
const grid = new THREE.GridHelper(4, 16, 0x475569, 0x1e293b); grid.visible = false; scene.add(grid);   // VR / desktop only
const ghost = new UR5eGhost();
scene.add(ghost.group);
const tcpPath = new PathLine(0x22d3ee, 4000, 0.003);    // robot TCP path (from status, robot frame -> XR)
const handPath = new PathLine(0xfb923c, 4000, 0.004);   // controller grip-position path (XR frame)
scene.add(tcpPath.line, handPath.line);
let pathEpoch = -1, lastEngaged = false, fkChecked = false;
// 3D options menu floating next to the right controller (fed from renderStatus, moved every frame)
const xrMenu = new XRMenu(scene);
window.__xrMenu = xrMenu;   // test hook (Playwright / console)
// Isaac camera windows stacked beside the menu (or in its place when closed), same hand anchor
const xrCams = new XRCams(scene);
window.__xrCams = xrCams;   // test hook
let rightPose = null;                                     // latest right grip pose {p:[x,y,z], q:[x,y,z,w]} or null
let headPose = { p: [0, 1.6, 0], q: [0, 0, 0, 1] };        // latest head pose (XR) / camera (2D test)
let lastFrameT = 0;
const frameDt = (now) => { const dt = lastFrameT ? (now - lastFrameT) / 1000 : 0; lastFrameT = now; return dt; };

function updateScene(d) {
  const ui = d.ui || {};
  if (ui.path_epoch != null && ui.path_epoch !== pathEpoch) {
    if (pathEpoch >= 0) { tcpPath.clear(); handPath.clear(); }
    pathEpoch = ui.path_epoch;
  }
  ghost.setBasePose(ui.ghost || null);
  if (ui.tcp_offset != null) ghost.setTcpOffset(ui.tcp_offset);
  if (d.q) ghost.setJoints(d.q);
  if (d.gripper != null) ghost.setGripper(d.gripper);
  if (ui.ghost && d.tcp) {
    if (d.engaged || /PLAYING/.test(d.state)) tcpPath.add(ghost.baseToWorld(d.tcp));
    else tcpPath.break();
    if (d.q && !fkChecked) {
      fkChecked = true;
      const t = ghost.tcpInBase();
      const err = Math.hypot(t.x - d.tcp[0], t.y - d.tcp[1], t.z - d.tcp[2]);
      log(`ghost FK check: tcp=(${t.x.toFixed(3)}, ${t.y.toFixed(3)}, ${t.z.toFixed(3)}) vs TCP=(${d.tcp.slice(0, 3).map(v => v.toFixed(3)).join(', ')}) err=${(err * 1000).toFixed(1)} mm`);
    }
  }
  lastEngaged = !!d.engaged;
}

// ---------------- WebXR ----------------
let xrSession = null, refSpace = null;

function controllerFrame(src, frame) {
  if (!src.gripSpace) return null;
  const pose = frame.getPose(src.gripSpace, refSpace);
  const gp = src.gamepad;
  const b = (i) => (gp && gp.buttons[i]) ? gp.buttons[i] : { value: 0, pressed: false };
  const c = {
    p: [0, 0, 0], q: [0, 0, 0, 1], tracked: !!pose && !pose.emulatedPosition,
    trigger: +b(0).value.toFixed(3), grip: +b(1).value.toFixed(3),
    stick: b(3).pressed, a: b(4).pressed, b: b(5).pressed,
    axes: gp ? [+(gp.axes[2] || 0).toFixed(3), +(gp.axes[3] || 0).toFixed(3)] : [0, 0],
  };
  if (pose) {
    const t = pose.transform;
    c.p = [+t.position.x.toFixed(5), +t.position.y.toFixed(5), +t.position.z.toFixed(5)];
    c.q = [+t.orientation.x.toFixed(6), +t.orientation.y.toFixed(6), +t.orientation.z.toFixed(6), +t.orientation.w.toFixed(6)];
  }
  return c;
}

const handV = new THREE.Vector3();
function onXRFrame(time, frame) {
  if (frame && xrSession) {
    const msg = { t: time / 1000, right: null, left: null, head: null };
    for (const src of xrSession.inputSources) {
      if (src.targetRayMode !== 'tracked-pointer') continue;
      const c = controllerFrame(src, frame);
      if (!c) continue;
      if (src.handedness === 'right') msg.right = c; else if (src.handedness === 'left') msg.left = c;
    }
    const vp = frame.getViewerPose(refSpace);
    if (vp) {
      const t = vp.transform;
      msg.head = { p: [t.position.x, t.position.y, t.position.z], q: [t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w] };
      headPose = msg.head;
    }
    if (msg.right) {
      $('hud-grip').textContent = (msg.right.grip * 100).toFixed(0) + '%';
      if (msg.right.grip > 0.5 && msg.right.tracked) handPath.add(handV.set(msg.right.p[0], msg.right.p[1], msg.right.p[2]));
      else handPath.break();
    }
    rightPose = msg.right && msg.right.tracked ? { p: msg.right.p, q: msg.right.q } : null;
    send(msg);
  }
  const dt = frameDt(time), rp = rightPose && rightPose.p, rq = rightPose && rightPose.q;
  xrMenu.update(dt, rp, rq, headPose.p, headPose.q);
  xrCams.update(dt, rp, rq, headPose.p, headPose.q, xrMenu.visible);
  renderer.render(scene, camera);
}

async function startXR(mode) {
  if (!navigator.xr) { log('WebXR not available (open this page in the Quest browser over https)'); return; }
  try {
    // Try the richest configuration first, then simpler ones: the Quest browser answers
    // NotSupportedError when a combination (e.g. dom-overlay + required local-floor in AR) is refused.
    const attempts = [
      { requiredFeatures: ['local-floor'], optionalFeatures: ['dom-overlay'], domOverlay: { root: hud } },
      { optionalFeatures: ['local-floor', 'dom-overlay'], domOverlay: { root: hud } },
      { requiredFeatures: ['local-floor'] },
      { optionalFeatures: ['local-floor'] },
      {},
    ];
    let lastErr = null;
    xrSession = null;
    for (let i = 0; i < attempts.length; i++) {
      try {
        xrSession = await navigator.xr.requestSession(mode, attempts[i]);
        log(`${mode}: session config #${i + 1} accepted (${JSON.stringify(Object.keys(attempts[i]))})`);
        break;
      } catch (e) { lastErr = e; log(`${mode}: config #${i + 1} refused: ${e && e.name}`); }
    }
    if (!xrSession) throw lastErr || new Error('no session');
    // reference space: local-floor if granted, else local (the bridge calibrates the frame anyway)
    const hasFloor = !xrSession.enabledFeatures || xrSession.enabledFeatures.includes('local-floor');
    renderer.xr.setReferenceSpaceType(hasFloor ? 'local-floor' : 'local');
    if (!hasFloor) log('local-floor not granted: using local reference space');
    await renderer.xr.setSession(xrSession);
    refSpace = renderer.xr.getReferenceSpace();
    grid.visible = (mode === 'immersive-vr');
    hud.hidden = false;
    hud.classList.add('xr');   // menu is the 3D panel in the headset, not the DOM copy
    xrSession.addEventListener('end', () => {
      hud.hidden = true; hud.classList.remove('xr'); xrSession = null; rightPose = null; grid.visible = false; log('session ended');
    });
    // headset taken off (without the keep-awake trick): the session goes 'hidden' and frames stop,
    // the bridge then holds the robot after stale_timeout. Tell it why.
    xrSession.addEventListener('visibilitychange', () => {
      log('visibility: ' + xrSession.visibilityState);
      send({ type: 'visibility', state: xrSession.visibilityState });
    });
    renderer.setAnimationLoop(onXRFrame);
    log(`${mode} session started`);
  } catch (e) { log(`XR start failed (${mode}): ${e && e.name}: ${e && e.message}`); }
}

$('btn-ar').onclick = () => startXR('immersive-ar');
$('btn-vr').onclick = () => startXR('immersive-vr');
if (navigator.xr) {
  navigator.xr.isSessionSupported('immersive-ar').then(ok => { $('btn-ar').disabled = !ok; log('immersive-ar supported: ' + ok); });
  navigator.xr.isSessionSupported('immersive-vr').then(ok => { $('btn-vr').disabled = !ok; log('immersive-vr supported: ' + ok); });
} else { $('btn-ar').disabled = $('btn-vr').disabled = true; }

// ---------------- 2D desktop test: mouse = controller, left button = grip ----------------
// keys: a / b buttons, t trigger, m = thumbstick click (menu), arrows = thumbstick, h = toggle head motion
let testOn = false;
$('btn-test').onclick = () => {
  testOn = !testOn; hud.hidden = !testOn;
  $('view').classList.toggle('on', testOn);
  if (testOn) {
    const w = $('view').clientWidth, h = $('view').clientHeight;
    renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
    grid.visible = true;
    renderer.setAnimationLoop((now) => {
      if (xrSession) return;
      const tgt = ghost.group.visible ? ghost.group.position : new THREE.Vector3(0, 0.8, 0);
      camera.position.set(tgt.x + 1.3, tgt.y + 1.2, tgt.z + 1.3); camera.lookAt(tgt);
      // fake controller = mouse (same pose as sent to the bridge), camera = the desktop view camera
      const dt = frameDt(now);
      xrMenu.update(dt, [mx, my, mz], [0, 0, 0, 1], camera.position, camera.quaternion);
      xrCams.update(dt, [mx, my, mz], [0, 0, 0, 1], camera.position, camera.quaternion, xrMenu.visible);
      renderer.render(scene, camera);
    });
  } else { grid.visible = false; renderer.setAnimationLoop(null); }
  log('2D test ' + (testOn ? 'on: drag = grip, wheel = up/down, t trigger, a/b, m = stick click (menu), arrows = stick, h = head moving' : 'off'));
};
let mx = 0, my = 0, mz = 0, mdown = false, headMoving = true, t2d = 0;
const keys = {};
window.addEventListener('mousemove', e => { mx = (e.clientX / innerWidth - 0.5) * 1.0; my = -(e.clientY / innerHeight - 0.5) * 1.0 + 1.0; });
window.addEventListener('mousedown', e => { if (e.button === 0) mdown = true; });
window.addEventListener('mouseup', () => { mdown = false; });
window.addEventListener('wheel', e => { mz += e.deltaY * 0.0005; });
window.addEventListener('keydown', e => { keys[e.key] = true; if (e.key === 'h') { headMoving = !headMoving; log('head ' + (headMoving ? 'moving' : 'still')); } });
window.addEventListener('keyup', e => { keys[e.key] = false; });
setInterval(() => {
  if (!testOn || xrSession) return;
  t2d += 1 / 72;
  const c = { p: [mx, my, mz], q: [0, 0, 0, 1], tracked: true, trigger: keys['t'] ? 1 : 0, grip: mdown ? 1 : 0,
              a: !!keys['a'], b: !!keys['b'], stick: !!keys['m'],
              axes: [(keys['ArrowRight'] ? 1 : 0) - (keys['ArrowLeft'] ? 1 : 0), (keys['ArrowDown'] ? 1 : 0) - (keys['ArrowUp'] ? 1 : 0)] };
  $('hud-grip').textContent = (c.grip * 100).toFixed(0) + '%';
  if (c.grip > 0.5) handPath.add(handV.set(mx, my, mz)); else handPath.break();
  const hp = headMoving ? [0.05 * Math.sin(t2d * 2), 1.6, 0.05 * Math.cos(t2d * 1.3)] : [0, 1.6, 0];
  send({ t: performance.now() / 1000, right: c, left: null, head: { p: hp, q: [0, 0, 0, 1] } });
}, 1000 / 72);
