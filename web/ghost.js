// Translucent UR5e "hologram" for the AR view + path lines.
// Kinematics and mesh offsets come from ur_description's ur5e.urdf (collision meshes, metres).
// The ghost group's frame is the UR *Base* frame (= URDF base_link_inertia), the same frame the
// bridge reports TCP poses in, so the bridge's ghost pose (XR <- base) places it directly.
import * as THREE from 'three';
import { STLLoader } from './vendor/STLLoader.js';

const PI = Math.PI;
const rpy = (r, p, y) => new THREE.Euler(r, p, y, 'ZYX');   // URDF rpy = Rz(y) Ry(p) Rx(r)

// [joint origin xyz, joint origin rpy, link mesh file, mesh origin xyz, mesh origin rpy]
const CHAIN = [
  { j: [[0, 0, 0.1625], [0, 0, 0]],            mesh: 'shoulder', m: [[0, 0, 0], [0, 0, PI]] },
  { j: [[0, 0, 0], [PI / 2, 0, 0]],            mesh: 'upperarm', m: [[0, 0, 0.138], [PI / 2, 0, -PI / 2]] },
  { j: [[-0.425, 0, 0], [0, 0, 0]],            mesh: 'forearm',  m: [[0, 0, 0.007], [PI / 2, 0, -PI / 2]] },
  { j: [[-0.3922, 0, 0.1333], [0, 0, 0]],      mesh: 'wrist1',   m: [[0, 0, -0.127], [PI / 2, 0, 0]] },
  { j: [[0, -0.0997, 0], [PI / 2, 0, 0]],      mesh: 'wrist2',   m: [[0, 0, -0.0997], [0, 0, 0]] },
  { j: [[0, 0.0996, 0], [PI / 2, PI, PI]],     mesh: 'wrist3',   m: [[0, 0, -0.0989], [PI / 2, 0, 0]] },
];
const FLANGE = [[0, 0, 0], [0, -PI / 2, -PI / 2]];
const TOOL0 = [[0, 0, 0], [PI / 2, 0, PI / 2]];

function placed(obj, xyz, e) {
  obj.position.set(xyz[0], xyz[1], xyz[2]);
  obj.quaternion.setFromEuler(rpy(e[0], e[1], e[2]));
  return obj;
}

export class UR5eGhost {
  constructor(opts = {}) {
    this.group = new THREE.Group();          // = UR Base frame
    this.group.visible = false;
    this.opacity = opts.opacity ?? 0.38;
    this.mat = new THREE.MeshStandardMaterial({
      color: opts.color ?? 0x60a5fa, transparent: true, opacity: this.opacity, depthWrite: false,
      side: THREE.DoubleSide, roughness: 0.6, metalness: 0.1,
    });
    this.matHi = this.mat.clone(); this.matHi.color.set(0xfbbf24);
    this.joints = [];                         // Object3D rotated about local Z by q[i]
    this.q = [0, 0, 0, 0, 0, 0];
    this.grip = 0;

    let parent = this.group;
    const baseMesh = placed(new THREE.Object3D(), [0, 0, 0], [0, 0, PI]);
    parent.add(baseMesh); this._loadMesh('base', baseMesh);
    for (const link of CHAIN) {
      const jo = placed(new THREE.Object3D(), link.j[0], link.j[1]);   // joint origin
      parent.add(jo);
      const ax = new THREE.Object3D();                                  // rotates about Z
      jo.add(ax);
      const mo = placed(new THREE.Object3D(), link.m[0], link.m[1]);
      ax.add(mo); this._loadMesh(link.mesh, mo);
      this.joints.push(ax);
      parent = ax;
    }
    const flange = placed(new THREE.Object3D(), FLANGE[0], FLANGE[1]);
    parent.add(flange);
    this.tool0 = placed(new THREE.Object3D(), TOOL0[0], TOOL0[1]);
    flange.add(this.tool0);
    this._buildGripper();
    this.setJoints(this.q);
  }

  _loadMesh(name, parent) {
    new STLLoader().load(`models/ur5e/${name}.stl`, (geom) => {
      geom.computeVertexNormals();
      const mesh = new THREE.Mesh(geom, this.mat);
      mesh.renderOrder = 2;
      parent.add(mesh);
    }, undefined, (e) => console.warn('ghost mesh', name, e));
  }

  // Robotiq 2F-85 proportions (coupling + body 0.10 m, fingers to 0.16 m, 85 mm stroke), fingers along
  // tool0 +Z, opening along tool0 X — the same orientation the Isaac gripper is aligned to.
  // The TCP frame (axes helper) sits at the fingertip centre: tool0 + tcpOffset along Z.
  _buildGripper() {
    const g = new THREE.Group();
    const coupling = new THREE.Mesh(new THREE.CylinderGeometry(0.0375, 0.0375, 0.012, 24), this.mat);
    coupling.rotation.x = Math.PI / 2; coupling.position.set(0, 0, 0.006); g.add(coupling);
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.072, 0.034, 0.088), this.mat);
    body.position.set(0, 0, 0.056); g.add(body);
    this.fingers = [];
    for (const s of [-1, 1]) {
      const f = new THREE.Group();
      const knuckle = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.024, 0.045), this.matHi);
      knuckle.position.set(0, 0, 0.0225); f.add(knuckle);
      const pad = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.022, 0.03), this.matHi);
      pad.position.set(-s * 0.004, 0, 0.052); f.add(pad);
      f.position.set(s * 0.0425, 0, 0.098);
      f.userData.side = s;
      g.add(f); this.fingers.push(f);
    }
    this.gripperGroup = g;
    this.tool0.add(g);
    this.tcpFrame = new THREE.Object3D();
    this.tcpFrame.add(new THREE.AxesHelper(0.05));
    this.tool0.add(this.tcpFrame);
    this.setTcpOffset(0.163);
    this.setGripper(0);
  }

  // d = distance flange -> fingertip centre (bridge ui.tcp_offset; 0.121 m for the Isaac 2F-85 asset).
  // The visual gripper is scaled along Z so its pads sit at the TCP.
  setTcpOffset(d) {
    if (!(d > 0.03)) return;
    this.tcpOffset = d;
    this.tcpFrame.position.set(0, 0, d);
    const k = d / 0.163;
    this.gripperGroup.scale.set(1, 1, k);
  }

  setGripper(v) {
    this.grip = Math.max(0, Math.min(1, v || 0));
    const half = 0.0425 * (1 - this.grip) + 0.004;
    for (const f of this.fingers) f.position.x = f.userData.side * half;
  }

  // TCP position in the base frame (for comparing against status.tcp)
  tcpInBase() {
    this.group.updateMatrixWorld(true);
    const v = new THREE.Vector3();
    this.tcpFrame.getWorldPosition(v);
    return this.group.worldToLocal(v);
  }

  setJoints(q) {
    if (!q || q.length < 6) return;
    this.q = q;
    for (let i = 0; i < 6; i++) this.joints[i].rotation.set(0, 0, q[i]);
  }

  // pose: {p:[3], q:[x,y,z,w]} in the XR reference space (bridge status ui.ghost), or null to hide
  setBasePose(pose) {
    if (!pose) { this.group.visible = false; return; }
    this.group.position.set(pose.p[0], pose.p[1], pose.p[2]);
    this.group.quaternion.set(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
    this.group.visible = true;
  }

  // Forward-kinematics check: tool0 position in the base frame (for comparing against status.tcp)
  tool0InBase() {
    this.group.updateMatrixWorld(true);
    const v = new THREE.Vector3();
    this.tool0.getWorldPosition(v);
    return this.group.worldToLocal(v);
  }

  // robot-frame point -> XR world point
  baseToWorld(p) {
    this.group.updateMatrixWorld(true);
    return this.group.localToWorld(new THREE.Vector3(p[0], p[1], p[2]));
  }
}

export class PathLine {
  constructor(color, max = 4000, minStep = 0.004) {
    this.max = max; this.minStep = minStep; this.n = 0;
    this.buf = new Float32Array(max * 3);
    this.geom = new THREE.BufferGeometry();
    this.geom.setAttribute('position', new THREE.BufferAttribute(this.buf, 3));
    this.geom.setDrawRange(0, 0);
    this.line = new THREE.Line(this.geom, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.9 }));
    this.line.frustumCulled = false;
    this.last = null;
  }
  add(v) {
    if (this.last && this.last.distanceTo(v) < this.minStep) return;
    if (this.n >= this.max) {                         // drop the oldest half
      this.buf.copyWithin(0, (this.max >> 1) * 3);
      this.n = this.max - (this.max >> 1);
    }
    this.buf.set([v.x, v.y, v.z], this.n * 3); this.n++;
    this.last = v.clone();
    this.geom.attributes.position.needsUpdate = true;
    this.geom.setDrawRange(0, this.n);
  }
  break() { this.last = null; }   // next point starts a new visual segment (we just allow a jump)
  clear() { this.n = 0; this.last = null; this.geom.setDrawRange(0, 0); }
}
