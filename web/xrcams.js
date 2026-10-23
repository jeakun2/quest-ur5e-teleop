// Isaac overlay cameras as small world-space windows next to the right hand: one framed plane per
// enabled camera (`ui.cams` {ceiling,side,ee}), fed from `cam/<name>.jpg` (320x240, ~8 Hz) through a
// canvas texture. They stack to the right of the options menu while it is open and take the menu's
// own spot when it is closed, so they stay by the controller either way. Placement/billboard
// helpers are shared with the menu panel (xrmenu.js).
import * as THREE from 'three';
import { MENU_OFFSET, PANEL_W, anchorPoint, billboardTo, headPoint, easeOut } from './xrmenu.js';

export const CAM_LABELS = { ceiling: '천장', side: '사이드', ee: 'EE' };
const ORDER = ['ceiling', 'side', 'ee'];

// ---- canvas layout (px): 16 px frame + 320x240 image + 24 px caption strip ----
const FR = 16, IW = 320, IH = 240, CAP_H = 24;
const CW = FR + IW + FR, CH = FR + IH + CAP_H + FR;      // 352 x 296
const WIN_W = 0.12;                                       // metres
const WIN_H = WIN_W * CH / CW;                            // ~0.101 (4:3 image plus the caption)
const GAP = 0.01;                                         // between stacked windows / to the menu
const STEP = WIN_H + GAP;                                 // next window this much lower
// beside the menu: menu right edge + gap + half a window, same height/depth as the menu
const SIDE_OFFSET = new THREE.Vector3(PANEL_W / 2 + GAP + WIN_W / 2, MENU_OFFSET.y, MENU_OFFSET.z);
const FADE_MS = 150, POLL_MS = 125;
const FONT = 'system-ui, -apple-system, "Segoe UI", Roboto, "Noto Sans KR", sans-serif';

function rrect(ctx, x, y, w, h, r) {
  r = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
}

class CamWindow {
  constructor(name, scene) {
    this.name = name;
    this.canvas = document.createElement('canvas');
    this.canvas.width = CW; this.canvas.height = CH;
    this.ctx = this.canvas.getContext('2d');
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.texture.minFilter = THREE.LinearFilter; this.texture.magFilter = THREE.LinearFilter;
    this.texture.generateMipmaps = false;
    this.material = new THREE.MeshBasicMaterial({ map: this.texture, transparent: true, depthTest: false,
                                                  depthWrite: false, side: THREE.DoubleSide, toneMapped: false, opacity: 0 });
    this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), this.material);
    this.mesh.renderOrder = 999;       // over the hologram, just under the menu panel
    this.mesh.frustumCulled = false;
    this.mesh.visible = false;
    this.mesh.name = 'xr-cam-' + name;
    scene.add(this.mesh);
    this.target = 0; this.progress = 0;   // 0 = hidden, 1 = shown
    this.pos = new THREE.Vector3(); this.snap = true;
    this.timer = null; this.busy = false; this.frame = null; this.noFrame = true; this.frames = 0;
    this._draw();
  }
  get active() { return this.target === 1 || this.progress > 0; }

  start() {
    if (this.target !== 1) { this.target = 1; if (this.progress === 0) this.snap = true; }
    this.mesh.visible = true;
    if (this.timer) return;
    this.frame = null; this.noFrame = true; this._draw();   // placeholder until the first frame arrives
    this.timer = setInterval(() => this._poll(), POLL_MS);
    this._poll();
  }
  stop() {
    this.target = 0;
    if (this.timer) { clearInterval(this.timer); this.timer = null; }
    this.busy = false;
  }
  _poll() {
    if (this.busy) return;               // one request in flight at a time (slow link: skip ticks)
    this.busy = true;
    const img = new Image();
    img.onload = () => {
      this.busy = false;
      if (!this.timer) return;           // turned off meanwhile
      this.frame = img; this.noFrame = false; this.frames++; this._draw();
    };
    img.onerror = () => {                // 404 until the stream is up: keep the last frame, dim the caption
      this.busy = false;
      if (!this.timer) return;
      if (!this.noFrame) { this.noFrame = true; this._draw(); }
    };
    img.src = `cam/${this.name}.jpg?t=${Date.now()}`;
  }

  // dt: seconds; tgt: world target position; cam: head position (Vector3)
  update(dt, tgt, cam) {
    const rate = 1000 / FADE_MS;
    this.progress = Math.min(1, Math.max(0, this.progress + (this.target === 1 ? 1 : -1) * rate * dt));
    if (this.progress === 0 && this.target === 0) { this.mesh.visible = false; this.material.opacity = 0; return; }
    if (tgt) { if (this.snap) { this.pos.copy(tgt); this.snap = false; } else this.pos.lerp(tgt, 0.25); }
    this.mesh.position.copy(this.pos);
    billboardTo(this.mesh, cam);
    const e = easeOut(this.progress), s = 0.8 + 0.2 * e;
    this.mesh.scale.set(WIN_W * s, WIN_H * s, 1);
    this.material.opacity = e;
  }

  _draw() {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, CW, CH);
    // thin dark frame with a light border
    rrect(ctx, 1, 1, CW - 2, CH - 2, 14);
    ctx.fillStyle = 'rgba(8, 12, 20, 0.9)'; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = 'rgba(255,255,255,0.22)'; ctx.stroke();
    // image area (last frame, or a hatched placeholder)
    ctx.save();
    rrect(ctx, FR, FR, IW, IH, 6); ctx.clip();
    if (this.frame) ctx.drawImage(this.frame, FR, FR, IW, IH);
    else {
      ctx.fillStyle = '#0b1220'; ctx.fillRect(FR, FR, IW, IH);
      ctx.strokeStyle = '#0f172a'; ctx.lineWidth = 10;
      for (let x = -IH; x < IW + IH; x += 20) { ctx.beginPath(); ctx.moveTo(FR + x, FR); ctx.lineTo(FR + x + IH, FR + IH); ctx.stroke(); }
      ctx.fillStyle = '#64748b'; ctx.font = `600 18px ${FONT}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.letterSpacing = '2px'; ctx.fillText('NO FRAME', FR + IW / 2, FR + IH / 2); ctx.letterSpacing = '0px';
    }
    ctx.restore();
    // caption strip: label left, LIVE / no frame right
    const cy = FR + IH + CAP_H / 2 + 1;
    ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
    ctx.fillStyle = this.noFrame ? '#94a3b8' : '#e2e8f0'; ctx.font = `600 16px ${FONT}`;
    ctx.fillText(CAM_LABELS[this.name] || this.name, FR + 4, cy);
    ctx.textAlign = 'right';
    if (this.noFrame) {
      ctx.fillStyle = '#64748b'; ctx.font = `500 13px ${FONT}`;
      ctx.fillText('no frame', CW - FR - 4, cy);
    } else {
      ctx.fillStyle = '#f87171'; ctx.font = `700 12px ${FONT}`; ctx.letterSpacing = '1.5px';
      ctx.fillText('LIVE', CW - FR - 4, cy); ctx.letterSpacing = '0px';
      ctx.beginPath(); ctx.arc(CW - FR - 44, cy, 4, 0, Math.PI * 2); ctx.fillStyle = '#ef4444'; ctx.fill();
    }
    this.texture.needsUpdate = true;
  }
}

export class XRCams {
  constructor(scene) {
    this.windows = ORDER.map(n => new CamWindow(n, scene));
    this.byName = Object.fromEntries(this.windows.map(w => [w.name, w]));
    this._cam = new THREE.Vector3(); this._base = new THREE.Vector3(); this._tgt = new THREE.Vector3();
  }
  // cams: {ceiling:bool, side:bool, ee:bool} or null (= all off)
  setCams(cams) {
    for (const w of this.windows) { if (cams && cams[w.name]) w.start(); else w.stop(); }
  }
  get visible() { return this.windows.some(w => w.active); }

  // dt in seconds. controllerPos/Quat: right grip pose (arrays or three objects) or null;
  // cameraPos/cameraQuat: head pose; menuVisible: stack beside the menu (true) or in its place (false).
  update(dt, controllerPos, controllerQuat, cameraPos, cameraQuat = null, menuVisible = false) {
    if (!this.windows.some(w => w.mesh.visible)) return;
    dt = Math.min(Math.max(dt || 0, 0), 0.1);
    const cam = headPoint(cameraPos, this._cam);
    const base = anchorPoint(this._base, menuVisible ? SIDE_OFFSET : MENU_OFFSET, controllerPos, controllerQuat, cam, cameraQuat);
    let slot = 0;
    for (const w of this.windows) {
      if (!w.mesh.visible) continue;
      let tgt = null;
      if (w.target === 1) { tgt = this._tgt.copy(base); tgt.y -= STEP * slot; slot++; }   // fading-out windows hold their spot
      w.update(dt, tgt, cam);
    }
  }
}
