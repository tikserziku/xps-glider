// Overview sheet A4 (Lithuanian only): 10 forms in 2 columns x 5 rows, the chosen form filled, site address + QR, author.
// Inline SVG in HTML -> vector PDF through the CDP Chromium (WEB-VIZ-RULES 2D-1). Closes only its own tab.
// Usage: node overview.mjs [chosenId=F07] [scaleOfChosen]   ->  ..\out\overview_A4.html + ..\out\overview_A4.pdf
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const out = path.resolve(here, '../out');
const chosen = process.argv[2] || 'F07';
const scale = +process.argv[3] || 0;
// optional author line: <project>/author.local.json (git-ignored); GLIDER_ANON=1 or no file -> no name on the sheet
const authorFile = path.resolve(here, '../author.local.json');
const who = (process.env.GLIDER_ANON !== '1' && fs.existsSync(authorFile)) ? JSON.parse(fs.readFileSync(authorFile, 'utf8')) : {};
const AUTHOR = who.author || '', GROUP = who.group || '';
const data = JSON.parse(fs.readFileSync(path.join(out, 'results.json'), 'utf8'));
const qr = JSON.parse(fs.readFileSync(path.join(out, 'qr.json'), 'utf8'));
const KIND = { conventional: 'klasikinė schema', tailless: 'beuodegė', canard: '„antis“', tandem: 'tandemas' };
const num = (v, d) => v.toFixed(d).replace('.', ',');
const esc = s => s.replace(/&/g, '&amp;').replace(/</g, '&lt;');

function silhouette(f, sel) {
  const W = 37, H = 36.5, sc = 0.128, cx = W / 2;
  const vlen = Math.max(...f.geom.V.map(p => p[0]));
  const y0 = (H - vlen * sc) / 2;
  const pts = f.geom.H.map(p => `${(cx + p[1] * sc).toFixed(2)},${(y0 + p[0] * sc).toFixed(2)}`).join(' ');
  return `<svg viewBox="0 0 ${W} ${H}" xmlns="http://www.w3.org/2000/svg">
    <line x1="${cx}" y1="${y0.toFixed(2)}" x2="${cx}" y2="${(y0 + vlen * sc).toFixed(2)}" stroke="#111" stroke-width="0.75" stroke-linecap="round"/>
    <polygon points="${pts}" fill="${sel ? '#111' : '#fff'}" stroke="#111" stroke-width="0.3" stroke-linejoin="round"/></svg>`;
}

function qrSvg(mm) {
  const q = 4, n = qr.size + 2 * q;
  let r = '';
  qr.rows.forEach((row, y) => {
    let x = 0;
    while (x < row.length) {
      if (row[x] === '1') { let e = x; while (e < row.length && row[e] === '1') e++; r += `<rect x="${x + q}" y="${y + q}" width="${e - x}" height="1"/>`; x = e; } else x++;
    }
  });
  return `<svg class="qr" viewBox="0 0 ${n} ${n}" style="width:${mm}mm;height:${mm}mm" shape-rendering="crispEdges" xmlns="http://www.w3.org/2000/svg"><rect width="${n}" height="${n}" fill="#fff"/><g fill="#000">${r}</g></svg>`;
}

const cards = data.forms.map(f => {
  const sel = f.id === chosen;
  const pick = sel ? `<div class="pick">PASIRINKTA${scale ? ` · gaminama 1:1 A1 lape, sparnų mostas ${Math.round(f.span_mm * scale)} mm` : ' · gaminama 1:1 A1 lape'}</div>` : '';
  return `<div class="card${sel ? ' sel' : ''}">${silhouette(f, sel)}<div class="txt">
    <div class="nm">${f.id} ${esc(f.name_lt)}<small>${KIND[f.kind]}</small></div>
    <div class="ds">${esc(f.why_lt)}</div>${pick}
    <div class="nu"><span><i>L</i>${num(f.range_sim_m, 1)} m</span><span><i>t</i>${num(f.time_sim_s, 2)} s</span><span><i>K</i>${num(f.LD, 1)}</span><span><i>m</i>${num(f.m_total_g, 1)} g</span></div></div></div>`;
}).join('\n');

const spans = data.forms.map(f => f.span_mm);
const html = `<!doctype html><html lang="lt"><head><meta charset="utf-8"><title>Sklandytuvas iš XPS – 10 formų</title>
<style>
@page { size: 210mm 297mm; margin: 0 }
* { box-sizing: border-box }
html, body { margin: 0; padding: 0; background: #fff }
body { font-family: "Segoe UI", Arial, sans-serif; color: #111; -webkit-print-color-adjust: exact; print-color-adjust: exact }
.page { position: relative; width: 210mm; height: 297mm; padding: 11mm 12mm 10mm; overflow: hidden }
header { display: flex; justify-content: space-between; align-items: flex-end; border-bottom: 0.6mm solid #111; padding-bottom: 2.5mm; height: 22mm }
h1 { font-size: 16pt; margin: 0 }
.sub { font-size: 9.5pt; color: #333; margin-top: 1mm }
.who { text-align: right; font-size: 12pt; font-weight: 700; line-height: 1.25; white-space: nowrap }
.who span { display: block; font-weight: 400; font-size: 10pt }
.grid { display: grid; grid-template-columns: 1fr 1fr; grid-template-rows: repeat(5, 40.5mm); grid-auto-flow: column; gap: 2.4mm 4mm; margin-top: 3.5mm }
.card { border: 0.25mm solid #777; border-radius: 2mm; display: flex; gap: 2mm; padding: 2mm 2.5mm; overflow: hidden }
.card svg { width: 37mm; height: 36.5mm; flex: none }
.card.sel { border: 0.8mm solid #111; background: #e3e7ea }
.txt { display: flex; flex-direction: column; min-width: 0; flex: 1 }
.nm { font-size: 11pt; font-weight: 700; line-height: 1.15 }
.nm small { display: block; font-weight: 400; font-size: 7.5pt; color: #444 }
.ds { font-size: 7.9pt; line-height: 1.27; margin-top: 0.8mm; color: #222 }
.pick { font-size: 7.6pt; font-weight: 700; margin-top: 0.8mm }
.nu { font-size: 7.6pt; font-weight: 700; margin-top: auto; display: flex; flex-wrap: wrap; gap: 0 2.6mm; white-space: nowrap }
.nu i { font-style: normal; font-weight: 400; color: #444; margin-right: 0.6mm }
footer { position: absolute; left: 12mm; right: 12mm; bottom: 10mm; height: 36mm; border-top: 0.6mm solid #111; padding-top: 2.5mm; display: flex; gap: 5mm }
footer .qr { flex: none }
footer .t { font-size: 8.4pt; line-height: 1.35 }
footer .url { font: 700 13pt Consolas, "Courier New", monospace; margin: 1mm 0 2mm }
footer b { font-weight: 700 }
</style></head><body><div class="page">
<header><div><h1>Sklandytuvas iš 5 mm XPS pakloto</h1><div class="sub">Aerodinamikos pamoka · 10 formų palyginimas · dvi plokščios detalės, sujungtos išpjovomis</div></div>
<div class="who">${AUTHOR}<span>${GROUP}</span></div></header>
<div class="grid">
${cards}
</div>
<footer>${qrSvg(32)}<div class="t">
<div>Interaktyvus puslapis – 3D modelis, skrydžio modeliavimas, šablonai spausdinimui:</div>
<div class="url">${esc(qr.url)}</div>
<div><b>L</b> – nuotolis metant iš 1,6 m aukščio · <b>t</b> – skrydžio laikas · <b>K</b> – aerodinaminė kokybė · <b>m</b> – masė su balastu.
Skaičiai apskaičiuoti A4 maketams (sparnų mostas ${Math.min(...spans)}–${Math.max(...spans)} mm, XPS 5 mm) sūkurių gardelės metodu; tai apytikslis įvertis. Užtušuota forma pasirinkta gaminti.</div>
</div></footer>
</div></body></html>`;
const htmlPath = path.join(out, 'overview_A4.html');
fs.writeFileSync(htmlPath, html, 'utf8');

// ---- print to PDF through CDP
const PORT = process.env.CDP_PORT || 9222;
const tab = await (await fetch(`http://127.0.0.1:${PORT}/json/new?${encodeURIComponent(pathToFileURL(htmlPath).href)}`, { method: 'PUT' })).json();
await new Promise(r => setTimeout(r, 2500));
const ws = new WebSocket(tab.webSocketDebuggerUrl);
await new Promise((ok, err) => { ws.onopen = ok; ws.onerror = err; });
let id = 0; const pending = new Map();
ws.onmessage = e => { const m = JSON.parse(e.data); if (m.id && pending.has(m.id)) { const p = pending.get(m.id); pending.delete(m.id); m.error ? p.err(new Error(JSON.stringify(m.error))) : p.ok(m.result); } };
const send = (method, params = {}) => new Promise((ok, err) => { const i = ++id; pending.set(i, { ok, err }); ws.send(JSON.stringify({ id: i, method, params })); });
try {
  const fit = await send('Runtime.evaluate', { returnByValue: true, expression: `JSON.stringify([...document.querySelectorAll('.card')].map(c => { const t = c.querySelector('.txt'); return [c.querySelector('.nm').textContent.slice(0, 3), t.scrollHeight > t.clientHeight + 1, c.scrollWidth > c.clientWidth + 1]; }).filter(x => x[1] || x[2]))` });
  const r = await send('Page.printToPDF', { paperWidth: 210 / 25.4, paperHeight: 297 / 25.4, marginTop: 0, marginBottom: 0, marginLeft: 0, marginRight: 0,
    printBackground: true, preferCSSPageSize: true, scale: 1 });
  fs.writeFileSync(path.join(out, 'overview_A4.pdf'), Buffer.from(r.data, 'base64'));
  console.log(JSON.stringify({ out: 'out/overview_A4.pdf', chosen, scale, cards_overflowing: JSON.parse(fit.result.value) }));
} finally {
  ws.close();
  await fetch(`http://127.0.0.1:${PORT}/json/close/${tab.id}`);
}
