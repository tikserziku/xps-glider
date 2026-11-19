// XPS Glider - FPV Lab. One offline page: 3D flight of the computed trajectory + HUD + live recalculation.
// Numbers: out/results.json (aero.py, exact model); GLB = view layer only (make_3d.py). Physics here is a port of aero.py.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { I18N } from './i18n.js';

const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const jget = id => JSON.parse(document.getElementById(id).textContent);

// ------------------------------------------------------------------ physics (port of src/aero.py)
const RHO = 1.225, NU = 1.5e-5, G = 9.81, CL_DESIGN_MAX = 0.45;
let DATA, A;

function simulate(p, v0, g0, h0) {
  const dt = 0.004, clmax = A.cl_max;
  const f = s => {
    const vx = s[2], vz = s[3], th = s[4], q = s[5];
    const V = Math.max(Math.hypot(vx, vz), 0.05), gam = Math.atan2(vz, vx), al = th - gam, qh = q * p.c / (2 * V);
    const clLin = p.CLa * al + p.CLd * p.delta + p.CLq * qh;
    const cl = Math.max(-clmax, Math.min(clmax, clLin));
    const aEq = Math.min(Math.abs(clLin) / p.CLa, 1.2);
    const cd = p.CD0 + p.kpol * p.CLa * Math.abs(cl) * Math.tan(aEq);
    const cm = p.Cma * al + p.Cmd * p.delta + p.Cmq * qh;
    const qd = 0.5 * RHO * V * V, L = qd * p.S * cl, D = qd * p.S * cd;
    return [vx, vz, (-D * Math.cos(gam) - L * Math.sin(gam)) / p.m, (-D * Math.sin(gam) + L * Math.cos(gam)) / p.m - G,
      q, qd * p.S * p.c * cm / p.Iyy];
  };
  let s = [0, h0, v0 * Math.cos(g0), v0 * Math.sin(g0), g0 + p.alpha0, 0];
  const add = (a, b, h) => a.map((v, i) => v + h * b[i]);
  const T = [0], X = [0], Z = [h0], TH = [s[4]], V = [v0], GA = [g0];
  let t = 0, zmax = h0;
  while (s[1] > 0 && t < 10) {
    const k1 = f(s), k2 = f(add(s, k1, dt / 2)), k3 = f(add(s, k2, dt / 2)), k4 = f(add(s, k3, dt));
    const n = s.map((v, i) => v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
    if (!n.every(Number.isFinite) || Math.abs(n[4]) > 40) break;
    let fr = 1;
    if (n[1] < 0) { fr = s[1] / (s[1] - n[1]); for (let i = 0; i < 6; i++) n[i] = s[i] + fr * (n[i] - s[i]); n[1] = 0; }
    s = n; t += dt * fr;
    zmax = Math.max(zmax, s[1]);
    T.push(t); X.push(s[0]); Z.push(s[1]); TH.push(s[4]); V.push(Math.hypot(s[2], s[3])); GA.push(Math.atan2(s[3], s[2]));
    if (fr < 1) break;
  }
  return { dt, T, X, Z, TH, V, GA, range: s[0], time: t, zmax };
}

function derive(f, o) {
  const j = f.js, k = o.k, rs = o.areal / 1000;     // sheet mass, kg/m2
  const S = j.S * k * k, c = j.c * k;
  const a0 = j.a0 * k * k, a1 = j.a1 * k ** 3, d0 = j.d0 * k * k, d1 = j.d1 * k ** 3, x0 = j.x0 * k ** 3, x1 = j.x1 * k ** 4;
  const am = j.am * k * k, dm = j.dm * k * k;
  // the planform scales with k, the half-lap slots stay 5 mm wide: gross part ~ k^2/k^3/k^4, slot part ~ k/k^2/k^3
  const area = k * k * j.Ag - k * j.dA;
  const mF = rs * area, xF = (k ** 3 * j.Agx - k * k * j.dAx) / area, i0 = rs * (k ** 4 * j.Ig - k ** 3 * j.dI), xb = j.xb * k;
  const swet = 2 * area, edge = j.edge_area * k, fc = j.flap_chord * k;
  const xnp = a1 / a0;
  const xc0 = xnp - A.sm_target * c;
  const mbAuto = Math.max(0, mF * (xF - xc0) / (xc0 - xb));
  const mb = mbAuto * o.balPct / 100;
  const m = mF + mb, xcg = (mF * xF + mb * xb) / m;
  const Iyy = i0 - mF * (2 * xcg * xF - xcg * xcg) + mb * (xb - xcg) ** 2;
  const SM = (xnp - xcg) / c;
  const K = {
    CLa: 2 * a0 / S, CLd: 2 * d0 / S, Cma: -2 * (a1 - xcg * a0) / (S * c), Cmd: -2 * (d1 - xcg * d0) / (S * c),
    CLq: 4 * (x0 - xcg * a0) / (c * S), Cmq: -4 * (x1 - xcg * x0 - xcg * a1 + xcg * xcg * a0) / (c * c * S),
    kaa: 2 * a0 / S, kad: 2 * (am + d0) / S, kdd: 2 * dm / S,
  };
  const cd0 = V => 1.328 / Math.sqrt(Math.max(V * c / NU, 100)) * swet / S + A.k_edge * edge / S;
  const point = (cl, al, de) => {
    const V = Math.sqrt(2 * m * G / (RHO * S * cl)), c0 = cd0(V), cd = c0 + K.kaa * al * al + K.kad * al * de + K.kdd * de * de;
    return { CL: cl, V, CD: cd, CD0: c0, LD: cl / cd, sink: V * Math.sin(Math.atan(cd / cl)), alpha: al, delta: de };
  };
  const stable = SM > 0.005;
  const det = K.CLa * K.Cmd - K.CLd * K.Cma;
  let best = null, status = 'ok', tr, delta = 0;
  if (stable) {
    for (let cl = 0.12; cl <= CL_DESIGN_MAX + 1e-9; cl += 0.01) {
      const p = point(cl, cl * K.Cmd / det, -cl * K.Cma / det);
      if (!best || p.LD > best.LD) best = p;
    }
    delta = best.delta + o.dOff;
    if (Math.abs(o.dOff) < 1e-9) tr = best;
    else {
      const al = -K.Cmd * delta / K.Cma, cl = K.CLa * al + K.CLd * delta;
      if (cl > A.cl_max) status = 'stall'; else if (cl < 0.08) status = 'dive';
      tr = point(Math.min(Math.max(cl, 0.08), A.cl_max), al, delta);
    }
  } else { status = 'unstable'; tr = point(0.3, 0.3 / K.CLa, 0); }
  const gTrim = -Math.atan(tr.CD / tr.CL);
  const p = { m, Iyy, S, c, ...K, CD0: tr.CD0, kpol: Math.max(0, tr.CD - tr.CD0) / (tr.CL * tr.CL), alpha0: tr.alpha, delta };
  const v0 = tr.V * o.v0Pct / 100, g0 = gTrim + o.angOff;
  const sim = simulate(p, v0, g0, o.h0);
  return {
    f, k, S, c, m, mb, mbAuto, mF, xcg, xnp, xb, SM, K, tr, best, status, sim, v0, g0, gTrim, delta, fc,
    span: f.span_mm * k, ws: m * 1000 / (S * 100), Re: tr.V * c / NU, bend: f.bending_use * m / (f.m_total_g / 1000),
    teMM: fc * Math.sin(delta) * 1000, alphaStall: A.cl_max / K.CLa,
  };
}

const defaults = (k = 1) => ({ k, areal: A.areal_kg_m2 * 1000, balPct: 100, dOff: 0, v0Pct: 100, angOff: 0, h0: A.h_launch_m });

// ------------------------------------------------------------------ state
const SOURCES = [
  ['Best Foamboard Chuck Glider Ever: The Sweepette Foamie — joshuawfinn', 'https://www.youtube.com/watch?v=SoFLbF-t0Rk'],
  ['F1N indoor hand launch glider. Tuning instructions — VILNIUS TECH AGAI', 'https://www.youtube.com/watch?v=TWpZZpMM0gQ'],
  ['Aerolab Exercise: FPG-9 foam plate glider — Academy of Model Aeronautics', 'https://www.youtube.com/watch?v=pNtew_VzzWg'],
  ['How to make a foam free flight glider — Joyplanes', 'https://www.youtube.com/watch?v=0qiCw5MZYXM'],
  ['Планер из подложки — ZloyGucul', 'https://www.youtube.com/watch?v=Io3ZObrV6U8'],
  ['Метательный планер «Микро 300» — Alex Kopert', 'https://www.youtube.com/watch?v=_gmVoEaKIS0'],
  ['Модель планера из потолочной плитки «УТКА» — андрей кушнир', 'https://www.youtube.com/watch?v=JqphX7y9Nis'],
  ['5mm Foam board glider quick build — KRVRCReview', 'https://www.youtube.com/watch?v=IrFOHjLkfBo'],
];
let lang = 'lt', fi = 0, mode = 'chase', slow = true, cur = null, auto = null, ui = null, CFG = null, GLBS = null;
const t = key => (I18N[lang][key] ?? key);
const fmt = (v, d = 1) => Number.isFinite(v) ? v.toFixed(d).replace('.', ',') : '—';   // decimal comma in both LT and RU
const fname = f => (lang === 'lt' ? f.name_lt : f.name_ru);

// ------------------------------------------------------------------ 3D
const three = {};
function init3d() {
  const canvas = $('#gl');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  const scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x05090e, 10, 42);
  const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 200);
  camera.position.set(-1.5, 1.9, 1.6);
  scene.add(new THREE.HemisphereLight(0xcfe4ff, 0x0a1218, 1.25));
  const sun = new THREE.DirectionalLight(0xffffff, 1.7); sun.position.set(-3, 6, 4); scene.add(sun);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(80, 30), new THREE.MeshStandardMaterial({ color: 0x08121b, roughness: 1 }));
  floor.rotation.x = -Math.PI / 2; floor.position.set(25, -0.002, 0); scene.add(floor);
  const grid = new THREE.GridHelper(80, 80, 0x1f7f60, 0x10303a); grid.position.set(25, 0, 0); scene.add(grid);
  // metre marks along the flight line
  const mk = [];
  for (let i = 0; i <= 30; i++) mk.push(i, 0.003, -1.2, i, 0.003, 1.2);
  const marks = new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(mk, 3)),
    new THREE.LineBasicMaterial({ color: 0x35f2a1, transparent: true, opacity: 0.55 }));
  const deco = new THREE.Group();                  // flight-line furniture, hidden in the 3D inspection mode
  scene.add(deco); deco.add(marks);
  for (let i = 1; i <= 30; i++) {
    const cv = document.createElement('canvas'); cv.width = 128; cv.height = 64;
    const g = cv.getContext('2d'); g.font = '700 40px Consolas, monospace'; g.fillStyle = '#35f2a1'; g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(i + ' m', 64, 34);
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(cv), transparent: true, opacity: 0.9, depthWrite: false }));
    sp.scale.set(0.5, 0.25, 1); sp.position.set(i, 0.16, -1.45); deco.add(sp);
  }
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 1, 10), new THREE.MeshBasicMaterial({ color: 0x3ad8ff, transparent: true, opacity: 0.7 }));
  deco.add(pole);
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.16, 0.2, 40), new THREE.MeshBasicMaterial({ color: 0xffb020, side: THREE.DoubleSide, transparent: true, opacity: 0.9 }));
  ring.rotation.x = -Math.PI / 2; deco.add(ring);
  const N = 2600;
  const trail = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(N * 3), 3)),
    new THREE.LineBasicMaterial({ color: 0x35f2a1 }));
  const ghost = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(N * 3), 3)),
    new THREE.LineBasicMaterial({ color: 0x35f2a1, transparent: true, opacity: 0.22 }));
  trail.frustumCulled = ghost.frustumCulled = false;
  deco.add(trail, ghost);
  const pivot = new THREE.Group(), inner = new THREE.Group();
  inner.rotation.y = Math.PI;                       // GLB (x aft, y up, z = -right)  ->  scene (x forward, y up, z right)
  pivot.add(inner); scene.add(pivot);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true; controls.enabled = false; controls.autoRotate = true; controls.autoRotateSpeed = 1.4;
  controls.minDistance = 0.15; controls.maxDistance = 8;
  Object.assign(three, { renderer, scene, camera, pole, ring, trail, ghost, deco, pivot, inner, controls, N, cache: new Map(), camPos: new THREE.Vector3(), camTgt: new THREE.Vector3(), snap: true });
  const resize = () => {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(canvas); resize();
}

function loadModel(id) {
  if (three.cache.has(id)) return three.cache.get(id);
  const pr = new Promise((ok, err) => {
    const bin = Uint8Array.from(atob(GLBS[id]), ch => ch.charCodeAt(0));
    new GLTFLoader().parse(bin.buffer, '', g => {
      const root = g.scene;
      root.traverse(o => {
        if (!o.isMesh) return;
        let isV = false;
        for (let n = o; n; n = n.parent) if (n.name === 'V') isV = true;
        o.material = new THREE.MeshStandardMaterial({ color: isV ? 0xff7a1a : 0xf3f3ea, roughness: 0.85, metalness: 0, side: THREE.DoubleSide, flatShading: true });
        const e = new THREE.LineSegments(new THREE.EdgesGeometry(o.geometry, 30), new THREE.LineBasicMaterial({ color: isV ? 0x6a2c00 : 0x1d6b55 }));
        o.add(e);
      });
      ok(root);
    }, err);
  });
  three.cache.set(id, pr);
  return pr;
}

async function showModel() {
  const f = DATA.forms[fi], want = f.id;
  const obj = await loadModel(want);
  if (DATA.forms[fi].id !== want) return;
  three.inner.clear(); three.inner.add(obj);
  placeModel();
}
function placeModel() {
  if (!cur) return;
  three.inner.scale.setScalar(cur.k);
  three.inner.position.set(cur.xcg, 0, 0);          // pivot = centre of gravity
}

function setPath() {
  const s = cur.sim, n = Math.min(s.X.length, three.N);
  for (const ln of [three.trail, three.ghost]) {
    const a = ln.geometry.attributes.position.array;
    for (let i = 0; i < n; i++) { a[i * 3] = s.X[i]; a[i * 3 + 1] = s.Z[i]; a[i * 3 + 2] = 0; }
    ln.geometry.attributes.position.needsUpdate = true;
  }
  three.ghost.geometry.setDrawRange(0, n);
  three.pole.scale.y = ui.h0; three.pole.position.set(0, ui.h0 / 2, 0);
  three.ring.position.set(s.range, 0.004, 0);
  three.snap = true;
}

const play = { t: 0, hold: 0 };
function stateAt(tt) {
  const s = cur.sim, n = s.T.length - 1;
  let i = Math.min(n - 1, Math.max(0, Math.floor(tt / s.dt)));
  if (n < 1) return { x: 0, z: ui.h0, th: 0, V: 0, ga: 0, i: 0 };
  const u = Math.min(1, Math.max(0, (tt - s.T[i]) / Math.max(1e-9, s.T[i + 1] - s.T[i])));
  const L = (a) => a[i] + (a[i + 1] - a[i]) * u;
  return { x: L(s.X), z: L(s.Z), th: L(s.TH), V: L(s.V), ga: L(s.GA), i: i + 1 };
}

let last = 0;
function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min(0.05, (now - last) / 1000 || 0.016); last = now;
  if (!cur) return;
  const { camera, pivot, controls } = three;
  const spanM = cur.span / 1000;
  if (mode === 'orbit') {
    pivot.position.set(0, 1, 0); pivot.rotation.set(0, 0, 0);
    controls.target.copy(pivot.position); controls.update();
    three.deco.visible = false;
  } else {
    three.deco.visible = true;
    if (play.t < cur.sim.time) play.t = Math.min(cur.sim.time, play.t + dt * (slow ? 0.3 : 1));
    else { play.hold += dt; if (play.hold > 1.8) { play.t = 0; play.hold = 0; three.snap = true; } }
    const st = stateAt(play.t);
    pivot.position.set(st.x, st.z, 0); pivot.rotation.set(0, 0, st.th);
    three.trail.geometry.setDrawRange(0, Math.min(st.i + 1, three.N));
    // portrait screens (phones): step back so the whole glider stays in the narrow frame
    const P = pivot.position, d = Math.max(0.6, 2.1 * spanM) * (camera.aspect < 1 ? Math.pow(camera.aspect, -0.8) : 1);
    const cp = three.camPos, ct = three.camTgt;
    let fov = 50;
    if (mode === 'fpv') {
      const c = Math.cos(st.th), s = Math.sin(st.th), up = 0.07 * cur.k, back = 0.02 * cur.k;
      cp.set(P.x - back * c - up * s, P.y - back * s + up * c, 0); ct.set(cp.x + c, cp.y + s, 0);
      camera.up.set(-s, c, 0); fov = 92; three.snap = true;
    } else if (mode === 'chase') {
      cp.set(P.x - d, P.y + 0.42 * d, 0.8 * d); ct.set(P.x + 0.25 * d, P.y, 0); camera.up.set(0, 1, 0);
    } else {
      const R = Math.max(cur.sim.range, 3), H = Math.max(cur.sim.zmax, ui.h0);
      const hf = 2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(25)) * camera.aspect);
      const dist = Math.max((R + 2.2) / 2 / Math.tan(hf / 2), (H + 1.2) / 2 / Math.tan(THREE.MathUtils.degToRad(25)));
      cp.set(R / 2, H / 2 + 0.25, dist); ct.set(R / 2, H / 2 - 0.1, 0); camera.up.set(0, 1, 0);
    }
    if (three.snap) { camera.position.copy(cp); three.look = ct.clone(); three.snap = false; }
    else { const a = 1 - Math.exp(-dt * 5); camera.position.lerp(cp, a); three.look.lerp(ct, a); }
    if (mode === 'fpv') camera.lookAt(ct); else camera.lookAt(three.look);
    if (camera.fov !== fov) { camera.fov = fov; camera.updateProjectionMatrix(); }
    hud(st);
  }
  three.renderer.render(three.scene, camera);
}

function setMode(m) {
  mode = m;
  $$('#modes .btn').forEach(b => b.classList.toggle('on', b.dataset.mode === m));
  $('#hero').classList.toggle('orbit', m === 'orbit');
  $('#hero').classList.toggle('nohorizon', m !== 'fpv');
  $('#modetag').textContent = m.toUpperCase();
  three.controls.enabled = m === 'orbit';
  three.camera.up.set(0, 1, 0);
  if (m === 'orbit') {
    const d = Math.max(0.55, 1.7 * cur.span / 1000);
    three.camera.position.set(-0.75 * d, 1 + 0.55 * d, 0.85 * d); three.camera.fov = 45; three.camera.updateProjectionMatrix();
  } else { three.snap = true; }
}

// ------------------------------------------------------------------ HUD
function buildLadder() {
  const g = $('#ladder'); let h = '';
  for (let p = -60; p <= 60; p += 5) {
    const y = -p * 3.2;
    if (p === 0) h += `<path d="M-130 ${y}H130" stroke="currentColor" stroke-width="1.6"/>`;
    else {
      const w = p % 10 === 0 ? 34 : 18;
      h += `<path d="M${-w} ${y}H${-10}M10 ${y}H${w}" stroke="currentColor" stroke-width="1" ${p < 0 ? 'stroke-dasharray="4 3"' : ''}/>`;
      if (p % 10 === 0) h += `<text x="${w + 4}" y="${y + 3}" font-size="8" fill="currentColor" font-family="Consolas,monospace">${p}</text>`;
    }
  }
  g.innerHTML = h;
}
function hud(st) {
  const deg = 180 / Math.PI, al = st.th - st.ga;
  $('#hV').textContent = fmt(st.V, 1);
  $('#hH').textContent = fmt(st.z, 2);
  $('#hA').textContent = fmt(al * deg, 1);
  const vs = -st.V * Math.sin(st.ga);
  $('#hS').textContent = fmt(vs, 2);
  $('#hL').textContent = fmt(st.x, 2);
  $('#hT').textContent = fmt(play.t, 2);
  $('#hK').textContent = vs > 0.05 ? fmt(st.V * Math.cos(st.ga) / vs, 1) : '—';
  $('#hP').textContent = fmt(st.th * deg, 1);
  const tt = play.t, mm = Math.floor(tt / 60), ss = tt - mm * 60;
  $('#clock').textContent = String(mm).padStart(2, '0') + ':' + ss.toFixed(2).padStart(5, '0');
  if (mode === 'fpv') {
    // ladder scale tied to the camera: SVG units per degree at the screen centre, so the 0 line lies on the real horizon
    const hz = $('#horizon'), cam = three.camera;
    const pxDeg = (three.renderer.domElement.clientHeight / 2) / Math.tan(cam.fov * Math.PI / 360) * Math.PI / 180;
    const u = pxDeg / (hz.clientWidth / 200);
    $('#ladder').setAttribute('transform', `translate(0 ${(st.th * deg * u).toFixed(2)}) scale(${(u / 3.2).toFixed(4)})`);
    $('#fpm').setAttribute('transform', `translate(0 ${(al * deg * u).toFixed(2)})`);
  }
  const w = $('#warn');
  let key = 'w_' + cur.status, cls = cur.status === 'ok' ? '' : (cur.status === 'dive' ? 'mid' : 'bad');
  if (cur.status === 'ok' && al > cur.alphaStall) { key = 'w_stall'; cls = 'bad'; }
  if (w.dataset.k !== key + lang) { w.dataset.k = key + lang; w.textContent = t(key); w.className = cls; }
}

// ------------------------------------------------------------------ panels
const GAUGES = [
  ['g_mass', 'g', d => d.m * 1000, 1, d => d.m * 1000 / (12 * d.k * d.k), () => ''],
  ['g_ballast', 'g', d => d.mb * 1000, 2, d => d.mb / d.m / 0.6, () => ''],
  ['g_ws', 'g/dm²', d => d.ws, 2, d => d.ws / 8, () => ''],
  ['g_sm', '%', d => d.SM * 100, 1, d => (d.SM + 0.1) / 0.4, d => d.SM <= 0.005 ? 'bad' : (d.SM < 0.04 || d.SM > 0.22 ? 'mid' : '')],
  ['g_v', 'm/s', d => d.tr.V, 2, d => d.tr.V / 7, () => ''],
  ['g_ld', '', d => d.status === 'unstable' ? NaN : d.tr.LD, 2, d => d.tr.LD / 7, d => d.status === 'ok' ? '' : 'mid'],
  ['g_sink', 'm/s', d => d.status === 'unstable' ? NaN : d.tr.sink, 2, d => d.tr.sink / 2, () => ''],
  ['g_range', 'm', d => d.sim.range, 2, d => d.sim.range / 12, d => d.status === 'unstable' ? 'bad' : ''],
  ['g_time', 's', d => d.sim.time, 2, d => d.sim.time / 4, () => ''],
  ['g_re', '', d => d.Re / 1000, 1, d => d.Re / 60000, () => '', '×10³'],
  ['g_bend', '%', d => d.bend * 100, 0, d => d.bend, d => d.bend > 1 ? 'bad' : (d.bend > 0.7 ? 'mid' : '')],
];
function buildGauges() {
  $('#gauges').innerHTML = GAUGES.map((g, i) => `<div class="g" id="g${i}"><small data-i18n="${g[0]}"></small><b></b><em>${g[6] || g[1]}</em><div class="bar"><i></i></div></div>`).join('');
}
function updGauges() {
  GAUGES.forEach((g, i) => {
    const el = $('#g' + i);
    $('b', el).textContent = fmt(g[2](cur), g[3]);
    $('.bar i', el).style.width = Math.max(0, Math.min(1, g[4](cur))) * 100 + '%';
    el.className = 'g ' + g[5](cur);
  });
  const st = $('#status'); st.textContent = '▸ ' + t('w_' + cur.status);
  st.className = cur.status === 'ok' ? 'ok' : (cur.status === 'dive' ? 'mid' : 'bad');
}
function updOutputs() {
  const size = cur.span <= 285 ? t('a4') : (cur.span >= 700 ? t('a1') : '');
  $('#oSize').textContent = `${Math.round(cur.span)} mm ${size ? '· ' + size : ''}`;
  $('#oAreal').textContent = `${Math.round(ui.areal)} g/m²`;
  $('#oBal').textContent = `${fmt(cur.mb * 1000, 2)} g ${ui.balPct === 100 ? '· ' + t('auto') : '· ' + ui.balPct + ' %'}`;
  const te = cur.teMM;
  $('#oEl').textContent = `${fmt(Math.abs(te), 1)} mm ${te <= 0 ? t('up') : t('down')}${Math.abs(ui.dOff) < 1e-9 ? ' · ' + t('auto') : ''}`;
  $('#oV0').textContent = `${fmt(cur.v0, 2)} m/s`;
  $('#oAng').textContent = `${fmt(cur.g0 * 180 / Math.PI, 0)}°${Math.abs(ui.angOff) < 1e-9 ? ' · ' + t('auto') : ''}`;
  $('#oH0').textContent = `${fmt(ui.h0, 2)} m`;
}

function drawTraj() {
  const W = 520, H = 250, ml = 40, mr = 10, mt = 10, mb = 28;
  const xm = Math.max(8, Math.ceil(Math.max(cur.sim.range, auto.sim.range) + 0.6));
  const zm = Math.max(2.2, Math.ceil((Math.max(cur.sim.zmax, auto.sim.zmax) + 0.2) * 2) / 2);
  const sx = x => ml + x / xm * (W - ml - mr), sz = z => H - mb - z / zm * (H - mt - mb);
  let h = '';
  for (let x = 0; x <= xm; x++) h += `<path d="M${sx(x)} ${sz(0)}V${sz(zm)}" stroke="#1b2a3a"/><text x="${sx(x)}" y="${H - 7}" font-size="13" fill="#8193a6" text-anchor="middle" font-family="Consolas,monospace">${x}</text>`;
  for (let z = 0; z <= zm + 1e-9; z += 0.5) h += `<path d="M${sx(0)} ${sz(z)}H${sx(xm)}" stroke="#1b2a3a"/><text x="${ml - 6}" y="${sz(z) + 4}" font-size="13" fill="#8193a6" text-anchor="end" font-family="Consolas,monospace">${fmt(z, 1)}</text>`;
  const line = (s, col, wd, dash) => {
    const step = Math.max(1, Math.floor(s.X.length / 260)); let d = '';
    for (let i = 0; i < s.X.length; i += step) d += (i ? 'L' : 'M') + sx(s.X[i]).toFixed(1) + ' ' + sz(Math.max(0, s.Z[i])).toFixed(1);
    d += 'L' + sx(s.range).toFixed(1) + ' ' + sz(0).toFixed(1);
    return `<path d="${d}" fill="none" stroke="${col}" stroke-width="${wd}" ${dash ? 'stroke-dasharray="5 4"' : ''} stroke-linejoin="round"/>`;
  };
  h += line(auto.sim, '#8193a6', 1.4, true) + line(cur.sim, '#35f2a1', 2.4, false);
  h += `<circle cx="${sx(0)}" cy="${sz(ui.h0)}" r="4" fill="#3ad8ff"/><circle cx="${sx(cur.sim.range)}" cy="${sz(0)}" r="4.5" fill="#ffb020"/>`;
  h += `<text x="${Math.min(W - 78, sx(cur.sim.range) + 8)}" y="${sz(0) - 9}" font-size="15" fill="#ffb020" font-family="Consolas,monospace" font-weight="700">${fmt(cur.sim.range, 2)} m</text>`;
  $('#traj').innerHTML = h;
}

function drawBal() {
  // side view: fuselage + fin outline, the horizontal part as a line, markers on the centreline
  const f = cur.f, W = 520, Vp = f.geom.V, hx = f.geom.H.map(p => p[0]), k = cur.k;
  const x1 = Math.max(...Vp.map(p => p[0])), zs = Vp.map(p => p[1]);
  const sc = Math.min((W - 60) / x1, 96 / (Math.max(...zs) - Math.min(...zs)));
  const off = (W - x1 * sc) / 2, X = x => off + x * sc, Y = z => 150 - z * sc;
  let h = `<path d="M${Vp.map(p => X(p[0]).toFixed(1) + ' ' + Y(p[1]).toFixed(1)).join('L')}Z" fill="rgba(255,122,26,.16)" stroke="#ff7a1a" stroke-width="1.5" stroke-linejoin="round"/>`;
  h += `<path d="M${X(Math.min(...hx))} ${Y(0)}H${X(Math.max(...hx))}" stroke="#f3f3ea" stroke-width="4" stroke-linecap="round"/>`;
  // the outline is drawn in mock-up mm; positions from derive() are metres at scale k -> mmk(); labels show real mm
  const mmk = v => v * 1000 / k;
  const mark = (xm, col, lab, up) => {
    const px = X(xm), y = Y(0), ye = up ? 44 : 224;
    return `<path d="M${px} ${ye}V${y}" stroke="${col}" stroke-width="1.8"/><circle cx="${px}" cy="${y}" r="6" fill="${col}" stroke="#05090e" stroke-width="1.5"/>` +
      `<text x="${Math.max(60, Math.min(W - 60, px))}" y="${up ? 34 : 242}" font-size="15" fill="${col}" text-anchor="middle" font-family="Consolas,monospace" font-weight="700">${lab}</text>`;
  };
  h += `<circle cx="${X(mmk(cur.xb))}" cy="${Y(0)}" r="${Math.max(5, Math.min(14, 5 + Math.sqrt(cur.mb / cur.m) * 15))}" fill="#ffb020" opacity=".92"/>`;
  const lcg = lang === 'lt' ? 'SC' : 'ЦТ', lnp = lang === 'lt' ? 'NT' : 'Ф';
  h += mark(mmk(cur.xnp), '#3ad8ff', `${lnp} ${Math.round(cur.xnp * 1000)} mm`, false);
  h += mark(mmk(cur.xcg), cur.SM > 0.005 ? '#35f2a1' : '#ff4d5e', `${lcg} ${Math.round(cur.xcg * 1000)} mm`, true);
  $('#bal').innerHTML = h;
}
function buildForms() {
  const S = 0.39;
  $('#forms').innerHTML = DATA.forms.map((f, i) => {
    const Hp = f.geom.H, Vp = f.geom.V, x1 = Math.max(...Vp.map(p => p[0]));
    const pts = Hp.map(p => `${(60 + p[1] * S).toFixed(1)},${(12 + p[0] * S).toFixed(1)}`).join(' ');
    return `<button class="fc" data-i="${i}"><span class="id">${f.id}</span>
      <svg viewBox="0 0 120 104"><path d="M60 12V${(12 + x1 * S).toFixed(1)}" stroke="#ff7a1a" stroke-width="2.2" stroke-linecap="round"/>
      <polygon points="${pts}" fill="rgba(243,243,234,.92)" stroke="#35f2a1" stroke-width=".8" stroke-linejoin="round"/></svg>
      <span class="pk" hidden></span><div class="nm"></div><div class="kd"></div><div class="nums"></div></button>`;
  }).join('');
  $$('#forms .fc').forEach(b => b.addEventListener('click', () => selectForm(+b.dataset.i)));
}
function updForms() {
  const rows = DATA.forms.map(f => derive(f, { ...defaults(ui.k), areal: ui.areal, h0: ui.h0 }));
  const rmax = Math.max(...rows.map(r => r.sim.range));
  $$('#forms .fc').forEach((b, i) => {
    const f = DATA.forms[i], r = rows[i];
    b.classList.toggle('on', i === fi);
    const pk = $('.pk', b); pk.hidden = f.id !== CFG.chosen; pk.textContent = t('picked');
    $('.nm', b).textContent = fname(f);
    $('.kd', b).textContent = t('k_' + f.kind);
    $('.nums', b).innerHTML = `<span>${fmt(r.sim.range, 1)}<i>m</i></span><span>${fmt(r.sim.time, 2)}<i>s</i></span><span>K ${fmt(r.tr.LD, 1)}</span>`;
  });
  $('#cmp').innerHTML = rows.map((r, i) => `<tr data-i="${i}" class="${i === fi ? 'on' : ''}"><td>${r.f.id} · ${fname(r.f)}</td><td>${fmt(r.m * 1000, 1)}</td><td>${fmt(r.mb * 1000, 2)}</td>
    <td>${fmt(r.tr.V, 2)}</td><td>${fmt(r.tr.LD, 2)}</td><td>${fmt(r.tr.sink, 2)}</td>
    <td><span class="rb" style="width:${(r.sim.range / rmax * 70).toFixed(0)}px"></span>${fmt(r.sim.range, 2)}</td><td>${fmt(r.sim.time, 2)}</td></tr>`).join('');
  $$('#cmp tr').forEach(tr => tr.addEventListener('click', () => { selectForm(+tr.dataset.i); $('#hero').scrollIntoView({ behavior: 'smooth' }); }));
  const f = DATA.forms[fi];
  $('#why').textContent = `${f.id} · ${fname(f)} — ${lang === 'lt' ? f.why_lt : f.why_ru}`;
  $('#formtag').textContent = `${f.id} · ${fname(f).toUpperCase()} · ${Math.round(cur.span)} mm`;
}

// ------------------------------------------------------------------ control flow
function recompute(restart = true) {
  const f = DATA.forms[fi];
  cur = derive(f, ui);
  auto = derive(f, { ...defaults(ui.k), areal: ui.areal, h0: ui.h0 });
  updOutputs(); updGauges(); drawTraj(); drawBal(); updForms(); placeModel(); setPath();
  if (restart) { play.t = 0; play.hold = 0; }
}
function readUI() {
  ui.k = +$('#iSize').value; ui.areal = +$('#iAreal').value; ui.balPct = +$('#iBal').value;
  ui.dOff = +$('#iEl').value * Math.PI / 180; ui.v0Pct = +$('#iV0').value; ui.angOff = +$('#iAng').value * Math.PI / 180; ui.h0 = +$('#iH0').value;
}
function writeUI() {
  $('#iSize').value = ui.k; $('#iAreal').value = ui.areal; $('#iBal').value = ui.balPct; $('#iEl').value = ui.dOff * 180 / Math.PI;
  $('#iV0').value = ui.v0Pct; $('#iAng').value = ui.angOff * 180 / Math.PI; $('#iH0').value = ui.h0;
}
function selectForm(i) {
  fi = i; recompute(); showModel();
  if (mode === 'orbit') setMode('orbit');
}
function setLang(l) {
  lang = l; document.documentElement.lang = l;
  $$('[data-i18n]').forEach(e => { e.textContent = t(e.dataset.i18n); });
  $$('#lang .btn').forEach(b => b.classList.toggle('on', b.dataset.lang === l));
  $('#slow').textContent = slow ? t('slow') + ' ×0,3' : t('realtime');
  try { localStorage.setItem('xpsglider.lang', l); } catch (e) { /* file:// may block storage */ }
  if (cur) { updOutputs(); updGauges(); drawBal(); updForms(); $('#warn').dataset.k = ''; }
}

function boot() {
  DATA = jget('data'); A = DATA.assumptions; GLBS = jget('glb'); CFG = jget('cfg');
  ui = defaults(1);
  fi = Math.max(0, DATA.forms.findIndex(f => f.id === CFG.chosen));     // the form chosen for the full-size build opens first
  try { const s = localStorage.getItem('xpsglider.lang'); if (s && I18N[s]) lang = s; } catch (e) { /* ignore */ }
  const q = new URLSearchParams(location.search).get('lang'); if (q && I18N[q]) lang = q;
  buildLadder(); buildGauges(); buildForms(); init3d();
  $('#src').innerHTML = SOURCES.map(s => `<a href="${s[1]}" target="_blank" rel="noopener">▸ ${s[0]}</a>`).join('');
  if (CFG.pdf_url) { const a = $('#pdf'); a.href = CFG.pdf_url; a.hidden = false; }
  if (CFG.repo_url) {
    const a = $('#repo'); a.href = CFG.repo_url; a.hidden = false;
    const b = $('#repo2'); b.href = CFG.repo_url; b.textContent = CFG.repo_url.replace(/^https?:\/\//, ''); $('#open').hidden = false;
  }
  writeUI(); recompute(); setLang(lang); setMode('chase'); showModel();
  $$('#lab input[type=range]').forEach(e => e.addEventListener('input', () => { readUI(); recompute(); }));
  $('#reset').addEventListener('click', () => { ui = defaults(1); writeUI(); recompute(); });
  $$('#modes .btn').forEach(b => b.addEventListener('click', () => setMode(b.dataset.mode)));
  $$('#lang .btn').forEach(b => b.addEventListener('click', () => setLang(b.dataset.lang)));
  $('#throw').addEventListener('click', () => { if (mode === 'orbit') setMode('chase'); play.t = 0; play.hold = 0; three.snap = true; });
  $('#slow').addEventListener('click', () => { slow = !slow; setLang(lang); });
  $('#dl').addEventListener('click', () => {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([window.__PAGE_SRC__], { type: 'text/html;charset=utf-8' }));
    a.download = 'xps-glider-fpv-lab.html'; document.body.appendChild(a); a.click();
    setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
  });
  requestAnimationFrame(frame);
  // numbers for the external check (verify.py compares them with aero.py)
  window.__selfcheck = () => DATA.forms.map(f => {
    const a = derive(f, defaults(1)), b = derive(f, defaults(2.8));
    return { id: f.id, m: a.m * 1000, mb: a.mb * 1000, V: a.tr.V, LD: a.tr.LD, range: a.sim.range, time: a.sim.time, sm: a.SM,
      m28: b.m * 1000, V28: b.tr.V, LD28: b.tr.LD, range28: b.sim.range, bend28: b.bend };
  });
  window.__app = { derive, defaults, get cur() { return cur; }, setMode, selectForm, setLang, three, play };
}
document.addEventListener('DOMContentLoaded', boot);
