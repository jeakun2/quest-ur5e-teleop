// World-space options menu for AR/VR: the bridge's `ui.menu.items` ({key,label,value,sel}) drawn
// on a canvas texture and floated next to the right controller, facing the user. Navigation is
// still the bridge's job (stick up/down, A select, B close); this only renders the list.
// Also the single source of the menu grouping (shared with the DOM menu in app.js).
import * as THREE from 'three';

// Options menu grouping: the bridge owns the item list, order and cursor; the page only groups
// rows by key prefix (a header is inserted whenever the group changes, so the order as sent is
// preserved) and renders on/off values as pills.
export const MENU_GROUPS = [
  ['동작', ['home', 'rot']],
  ['보정', ['tutorial', 'calib', 'calib3']],
  ['축 고정', ['lock_x', 'lock_y', 'lock_z', 'lock_rot']],
  ['녹화 / 매핑', ['record', 'replay', 'scale', 'map_mode']],
  ['Isaac 카메라 / 시점', ['cam_ceiling', 'cam_side', 'cam_ee', 'view']],
  ['AR', ['ghost', 'path_clear']],
  ['', ['close']],
];
export const menuGroupOf = (key) => {
  for (const [name, keys] of MENU_GROUPS) {
    if (keys.includes(key)) return name;
    if (keys.some(k => key.startsWith(k + '_'))) return name;   // e.g. lock_*, cam_*
  }
  return '';
};
export const PILL_ON = ['켜짐', '표시 중', 'on', 'ON'], PILL_OFF = ['꺼짐', '숨김', 'off', 'OFF'];

// ---- canvas layout (px) ----
const W = 512, PAD = 26, HEADER_H = 62, HINT_H = 46, ROW_H = 52, GRP_H = 30, CLOSE_GAP = 12;
const MAX_LIST_H = 14 * ROW_H;                 // ~14 rows visible, the rest scrolls
export const PANEL_W = 0.22;                   // metres
const SHOW_MS = 180, HIDE_MS = 120;
export const MENU_OFFSET = new THREE.Vector3(0.0, 0.10, -0.06);   // from the grip, in controller space
const FONT = 'system-ui, -apple-system, "Segoe UI", Roboto, "Noto Sans KR", sans-serif';
const MONO = 'ui-monospace, Menlo, Consolas, monospace';
export const easeOut = (x) => 1 - Math.pow(1 - x, 3);

// ---- shared placement helpers (also used by xrcams.js) ----
export const toV3 = (v, out) => Array.isArray(v) ? out.set(v[0], v[1], v[2]) : out.copy(v);
export const toQ = (q, out) => Array.isArray(q) ? out.set(q[0], q[1], q[2], q[3]) : out.copy(q);
const _hv = new THREE.Vector3(), _hq = new THREE.Quaternion();
// Head position as a Vector3 (default eye height at the origin when unknown).
export const headPoint = (cameraPos, out) => cameraPos ? toV3(cameraPos, out) : out.set(0, 1.6, 0);
// World-space anchor for a hand-attached panel: `offset` in controller (grip) space when a tracked
// controller pose is given; otherwise 0.5 m in front of the head (horizontal forward, eye height),
// shifted sideways by offset.x so panels keep their left/right arrangement.
export function anchorPoint(out, offset, controllerPos, controllerQuat, cam, cameraQuat) {
  if (controllerPos && controllerQuat) {
    return out.copy(offset).applyQuaternion(toQ(controllerQuat, _hq)).add(toV3(controllerPos, _hv));
  }
  _hv.set(0, 0, -1);
  if (cameraQuat) { _hv.applyQuaternion(toQ(cameraQuat, _hq)); _hv.y = 0; }
  if (_hv.lengthSq() < 1e-6) _hv.set(0, 0, -1);
  _hv.normalize();
  out.copy(_hv).multiplyScalar(0.5).add(cam);
  if (offset.x) out.x += -_hv.z * offset.x, out.z += _hv.x * offset.x;   // right = fwd x up
  return out;
}
// Billboard: yaw towards the camera, pitch limited so the panel stays roughly upright.
export function billboardTo(mesh, cam) {
  const p = mesh.position;
  const dx = cam.x - p.x, dy = cam.y - p.y, dz = cam.z - p.z;
  const yaw = Math.atan2(dx, dz);
  const pitch = Math.max(-0.5, Math.min(0.5, Math.atan2(dy, Math.hypot(dx, dz))));
  mesh.rotation.set(-pitch, yaw, 0, 'YXZ');
}

function rrect(ctx, x, y, w, h, r) {
  r = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
}
function fitText(ctx, s, maxW) {
  if (ctx.measureText(s).width <= maxW) return s;
  let lo = 0, hi = s.length;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (ctx.measureText(s.slice(0, m) + '…').width <= maxW) lo = m; else hi = m - 1; }
  return s.slice(0, lo) + '…';
}
export class XRMenu {
  constructor(scene) {
    this.canvas = document.createElement('canvas');
    this.canvas.width = W; this.canvas.height = HEADER_H + ROW_H + HINT_H;
    this.ctx = this.canvas.getContext('2d');
    this.texture = null;
    this.material = new THREE.MeshBasicMaterial({ transparent: true, depthTest: false, depthWrite: false,
                                                  side: THREE.DoubleSide, toneMapped: false, opacity: 0 });
    this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), this.material);
    this.mesh.renderOrder = 1000;      // over the hologram and paths
    this.mesh.frustumCulled = false;
    this.mesh.visible = false;
    this.mesh.name = 'xr-menu';
    scene.add(this.mesh);
    this.items = []; this.title = '옵션'; this.key = null; this.scroll = 0;
    this.progress = 0; this.target = 0;   // 0 = hidden, 1 = shown (eased for scale/opacity)
    this.pos = new THREE.Vector3(); this.snap = true;
    this._v = new THREE.Vector3(); this._cam = new THREE.Vector3();
    this._newTexture();
    this._draw();
  }
  get visible() { return this.target === 1 || this.progress > 0; }

  setItems(items, title) {
    this.items = Array.isArray(items) ? items : [];
    if (title) this.title = title;
    const key = JSON.stringify([this.title, this.items.map(it => [it.key, it.label, it.value, !!it.sel])]);
    if (key === this.key) return;
    this.key = key;
    this._draw();
  }
  show() { if (this.target !== 1) { this.target = 1; if (this.progress === 0) this.snap = true; } this.mesh.visible = true; }
  hide() { this.target = 0; }

  // dt in seconds. controllerPos/Quat: right grip pose in XR space (arrays or three objects) or null;
  // cameraPos/cameraQuat: head pose (quat optional, used only for the no-controller fallback).
  update(dt, controllerPos, controllerQuat, cameraPos, cameraQuat = null) {
    if (!this.mesh.visible) return;
    dt = Math.min(Math.max(dt || 0, 0), 0.1);
    const rate = this.target === 1 ? 1000 / SHOW_MS : 1000 / HIDE_MS;
    this.progress += (this.target === 1 ? 1 : -1) * rate * dt;
    this.progress = Math.min(1, Math.max(0, this.progress));
    if (this.progress === 0 && this.target === 0) { this.mesh.visible = false; this.material.opacity = 0; return; }

    const cam = headPoint(cameraPos, this._cam);
    // next to the grip, or (no controller) 0.5 m in front of the head at eye height
    const tgt = anchorPoint(this._v, MENU_OFFSET, controllerPos, controllerQuat, cam, cameraQuat);
    if (this.snap) { this.pos.copy(tgt); this.snap = false; } else this.pos.lerp(tgt, 0.25);
    this.mesh.position.copy(this.pos);
    billboardTo(this.mesh, cam);

    const e = easeOut(this.progress);
    const s = 0.6 + 0.4 * e;
    const h = PANEL_W * this.canvas.height / this.canvas.width;
    this.mesh.scale.set(PANEL_W * s, h * s, 1);
    this.material.opacity = e;
  }

  // ---- drawing ----
  _newTexture() {
    if (this.texture) this.texture.dispose();
    const t = new THREE.CanvasTexture(this.canvas);
    t.colorSpace = THREE.SRGBColorSpace;
    t.minFilter = THREE.LinearFilter; t.magFilter = THREE.LinearFilter; t.generateMipmaps = false;
    this.texture = t; this.material.map = t; this.material.needsUpdate = true;
  }
  _rows() {
    const rows = []; let grp = null, y = 0;
    for (const it of this.items) {
      const g = menuGroupOf(it.key);
      if (g !== grp) { grp = g; if (g) { rows.push({ t: 'grp', name: g, y, h: GRP_H }); y += GRP_H; } }
      const close = it.key === 'close';
      const h = ROW_H + (close ? CLOSE_GAP : 0);
      rows.push({ t: 'item', it, y, h, close });
      y += h;
    }
    return { rows, contentH: y };
  }
  _draw() {
    const { rows, contentH } = this._rows();
    const listH = Math.max(ROW_H, Math.min(contentH, MAX_LIST_H));
    // keep the selected row in view (like scrollIntoView 'nearest')
    const sel = rows.find(r => r.t === 'item' && r.it.sel);
    if (sel) {
      const top = sel.y - (rows[rows.indexOf(sel) - 1]?.t === 'grp' ? GRP_H : 0), bot = sel.y + sel.h;
      if (top < this.scroll) this.scroll = top;
      if (bot > this.scroll + listH) this.scroll = bot - listH;
    }
    this.scroll = Math.max(0, Math.min(this.scroll, Math.max(0, contentH - listH)));

    const H = HEADER_H + listH + HINT_H;
    const resized = this.canvas.height !== H;
    if (resized) this.canvas.height = H;
    const ctx = this.ctx;
    ctx.clearRect(0, 0, W, H);
    ctx.textBaseline = 'middle';

    // panel: dark translucent rounded rectangle with a thin light border
    rrect(ctx, 1, 1, W - 2, H - 2, 26);
    ctx.fillStyle = 'rgba(8, 12, 20, 0.86)'; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = 'rgba(255,255,255,0.22)'; ctx.stroke();

    // header
    ctx.fillStyle = '#f1f5f9'; ctx.font = `700 26px ${FONT}`; ctx.textAlign = 'left';
    ctx.fillText(this.title, PAD, HEADER_H / 2 + 1);
    if (contentH > listH) {   // scroll hint: position of the window within the list
      ctx.fillStyle = '#64748b'; ctx.font = `600 15px ${FONT}`; ctx.textAlign = 'right';
      ctx.fillText(`${this.scroll > 0 ? '▲ ' : ''}${this.scroll + listH < contentH ? '▼' : ''}`, W - PAD, HEADER_H / 2 + 1);
    }
    ctx.fillStyle = 'rgba(255,255,255,0.14)'; ctx.fillRect(PAD - 8, HEADER_H - 1, W - 2 * PAD + 16, 1);

    // list (clipped, scrolled)
    ctx.save();
    ctx.beginPath(); ctx.rect(0, HEADER_H, W, listH); ctx.clip();
    const y0 = HEADER_H - this.scroll;
    for (const r of rows) {
      const y = y0 + r.y;
      if (y + r.h < HEADER_H || y > HEADER_H + listH) continue;
      if (r.t === 'grp') {
        ctx.fillStyle = '#94a3b8'; ctx.font = `700 15px ${FONT}`; ctx.textAlign = 'left';
        ctx.letterSpacing = '1.5px';
        ctx.fillText(r.name.toUpperCase(), PAD + 2, y + GRP_H / 2 + 4);
        ctx.letterSpacing = '0px';
        continue;
      }
      const it = r.it, ry = y + (r.close ? CLOSE_GAP : 0);
      if (r.close) {   // dashed divider above "close"
        ctx.save(); ctx.setLineDash([5, 5]); ctx.strokeStyle = 'rgba(255,255,255,0.16)'; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(PAD - 8, y + CLOSE_GAP / 2 + .5); ctx.lineTo(W - PAD + 8, y + CLOSE_GAP / 2 + .5); ctx.stroke(); ctx.restore();
      }
      if (it.sel) {
        rrect(ctx, PAD - 8, ry + 3, W - 2 * PAD + 16, ROW_H - 6, 10);
        ctx.fillStyle = 'rgba(59, 130, 246, 0.36)'; ctx.fill();
        rrect(ctx, PAD - 16, ry + 9, 5, ROW_H - 18, 2.5);
        ctx.fillStyle = '#60a5fa'; ctx.fill();
      }
      // value (right): on/off pills, other values plain
      const v = it.value == null ? '' : String(it.value);
      let valW = 0;
      if (v) {
        const on = PILL_ON.includes(v), off = PILL_OFF.includes(v);
        ctx.textAlign = 'right';
        if (on || off) {
          ctx.font = `600 15px ${FONT}`;
          const tw = ctx.measureText(v).width, pw = tw + 24, ph = 28, px = W - PAD - pw, py = ry + ROW_H / 2 - ph / 2;
          rrect(ctx, px, py, pw, ph, 14);
          ctx.fillStyle = on ? 'rgba(34, 197, 94, 0.22)' : 'rgba(148, 163, 184, 0.14)'; ctx.fill();
          ctx.lineWidth = 1.5; ctx.strokeStyle = on ? 'rgba(74, 222, 128, 0.6)' : 'rgba(148, 163, 184, 0.4)'; ctx.stroke();
          ctx.fillStyle = on ? '#4ade80' : '#94a3b8';
          ctx.fillText(v, W - PAD - 12, ry + ROW_H / 2 + 1);
          valW = pw;
        } else {
          ctx.font = `400 17px ${MONO}`; ctx.fillStyle = it.sel ? '#e2e8f0' : '#cbd5e1';
          const s = fitText(ctx, v, (W - 2 * PAD) * 0.5);
          ctx.fillText(s, W - PAD, ry + ROW_H / 2 + 1);
          valW = ctx.measureText(s).width;
        }
      }
      // label (left)
      ctx.textAlign = 'left';
      ctx.font = `${it.sel ? 700 : 500} 22px ${FONT}`; ctx.fillStyle = it.sel ? '#ffffff' : (r.close ? '#cbd5e1' : '#e2e8f0');
      ctx.fillText(fitText(ctx, String(it.label), W - 2 * PAD - valW - 18), PAD + 4, ry + ROW_H / 2 + 1);
    }
    ctx.restore();

    // hint
    const hy = HEADER_H + listH;
    ctx.fillStyle = 'rgba(255,255,255,0.14)'; ctx.fillRect(PAD - 8, hy, W - 2 * PAD + 16, 1);
    ctx.textAlign = 'center'; ctx.font = `500 16px ${FONT}`; ctx.fillStyle = '#94a3b8';
    ctx.fillText('스틱 ▲▼ 이동 · A 선택 · B 닫기', W / 2, hy + HINT_H / 2 + 1);

    // a resized canvas needs a fresh GPU texture (texStorage keeps the old size), else just re-upload
    if (resized || !this.texture) this._newTexture(); else this.texture.needsUpdate = true;
  }
}
