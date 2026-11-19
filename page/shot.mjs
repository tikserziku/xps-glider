// Open the built page in an own CDP Chromium tab (port 9222), collect JS errors, run window.__selfcheck(),
// take screenshots at 1280x720 and 1920x1080 in every camera mode and of the main sections. Closes only its own tab.
// Usage: node shot.mjs <page.html | url> <outDir>
import { writeFileSync, mkdirSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

const PORT = process.env.CDP_PORT || 9222;
const [target, outDir] = process.argv.slice(2);
if (!target || !outDir) { console.error('usage: node shot.mjs <page.html|url> <outDir>'); process.exit(2); }
mkdirSync(outDir, { recursive: true });
const url = /^https?:/.test(target) ? target : pathToFileURL(resolve(target)).href;
const sleep = ms => new Promise(r => setTimeout(r, ms));

const tab = await (await fetch(`http://127.0.0.1:${PORT}/json/new?about:blank`, { method: 'PUT' })).json();
const ws = new WebSocket(tab.webSocketDebuggerUrl);
await new Promise((ok, err) => { ws.onopen = ok; ws.onerror = err; });
let id = 0; const pending = new Map(); const errors = [];
ws.onmessage = e => {
  const m = JSON.parse(e.data);
  if (m.id && pending.has(m.id)) { const p = pending.get(m.id); pending.delete(m.id); m.error ? p.err(new Error(JSON.stringify(m.error))) : p.ok(m.result); }
  else if (m.method === 'Runtime.exceptionThrown') errors.push('EXC ' + (m.params.exceptionDetails.exception?.description || m.params.exceptionDetails.text));
  else if (m.method === 'Runtime.consoleAPICalled' && (m.params.type === 'error' || m.params.type === 'warning'))
    errors.push(m.params.type.toUpperCase() + ' ' + m.params.args.map(a => a.value ?? a.description ?? '').join(' ').slice(0, 300));
  else if (m.method === 'Log.entryAdded' && m.params.entry.level === 'error') errors.push('LOG ' + m.params.entry.text.slice(0, 300));
};
const send = (method, params = {}) => new Promise((ok, err) => { const i = ++id; pending.set(i, { ok, err }); ws.send(JSON.stringify({ id: i, method, params })); });
const ev = async expr => {
  const r = await send('Runtime.evaluate', { expression: expr, returnByValue: true, awaitPromise: true });
  if (r.exceptionDetails) { errors.push('EVAL ' + (r.exceptionDetails.exception?.description || r.exceptionDetails.text)); return null; }
  return r.result?.value;
};
const shot = async name => {
  const r = await send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(join(outDir, name + '.png'), Buffer.from(r.data, 'base64'));
};
const size = (w, h) => send('Emulation.setDeviceMetricsOverride', { width: w, height: h, deviceScaleFactor: 1, mobile: false });

try {
  await send('Runtime.enable'); await send('Log.enable'); await send('Page.enable');
  await send('Page.bringToFront');
  await size(1280, 720);
  await send('Page.navigate', { url });
  await sleep(4500);
  const sc = await ev('JSON.stringify(window.__selfcheck ? window.__selfcheck() : null)');
  writeFileSync(join(outDir, 'selfcheck.json'), sc || 'null', 'utf8');
  const info = await ev(`JSON.stringify({ title: document.title, lang: document.documentElement.lang, forms: document.querySelectorAll('#forms .fc').length,
    gauges: document.querySelectorAll('#gauges .g').length, rows: document.querySelectorAll('#cmp tr').length,
    meshes: (() => { let n = 0; window.__app.three.inner.traverse(o => { if (o.isMesh) n++; }); return n; })(),
    src_kb: Math.round((window.__PAGE_SRC__ || '').length / 1024), src_has_app: (window.__PAGE_SRC__ || '').includes('__selfcheck'),
    src_ends: (window.__PAGE_SRC__ || '').slice(-16) })`);
  console.log('INFO', info);
  // what the "download HTML" button saves - written to disk so it can be opened and checked as a standalone file
  const src = await ev('window.__PAGE_SRC__ || ""');
  if (src) writeFileSync(join(outDir, 'downloaded.html'), src, 'utf8');
  // mid-flight frames in every mode (the flight loops; freeze time by setting play.t)
  for (const m of ['chase', 'fpv', 'side']) {
    await ev(`window.__app.setMode('${m}'); window.__app.play.t = window.__app.cur.sim.time * 0.45; window.__app.play.hold = 0; window.__app.three.snap = true; 1`);
    await sleep(700); await shot('1280_' + m);
  }
  await ev(`window.__app.setMode('orbit'); 1`); await sleep(1500); await shot('1280_orbit');
  // instant scroll (the page uses smooth scrolling, which would still be moving when the shot is taken)
  const go = (sel, off = 70) => ev(`(() => { const e = document.querySelector('${sel}'); const s = e.closest('section') || e;
    window.scrollTo({ top: s.getBoundingClientRect().top + window.scrollY - ${off}, behavior: 'instant' }); return 1; })()`);
  await ev(`window.__app.setMode('chase'); 1`);
  await go('#forms'); await sleep(600); await shot('1280_forms');
  await go('#lab'); await sleep(600); await shot('1280_lab');
  await ev(`document.querySelector('#iBal').value = 20; document.querySelector('#iBal').dispatchEvent(new Event('input')); 1`);
  await sleep(500); await shot('1280_lab_unstable');
  await ev(`document.querySelector('#reset').click(); window.__app.setLang('ru'); 1`);
  await go('#cmp'); await sleep(600); await shot('1280_cmp_ru');
  await go('.grid3'); await sleep(500); await shot('1280_why_ru');
  await ev(`window.__app.setLang('lt'); window.scrollTo({ top: 0, behavior: 'instant' }); 1`);
  await size(1920, 1080); await sleep(900);
  await ev(`window.__app.play.t = window.__app.cur.sim.time * 0.5; window.__app.three.snap = true; 1`); await sleep(700); await shot('1920_chase');
  await go('#lab'); await sleep(600); await shot('1920_lab');
  await size(390, 800); await ev(`window.scrollTo({ top: 0, behavior: 'instant' }); 1`); await sleep(1200); await shot('390_hero');
  await go('#lab'); await sleep(600); await shot('390_lab');
  console.log(errors.length ? 'ERRORS\n' + errors.join('\n') : 'NO JS ERRORS');
} finally {
  await send('Emulation.clearDeviceMetricsOverride').catch(() => {});
  ws.close();
  await fetch(`http://127.0.0.1:${PORT}/json/close/${tab.id}`);
}
