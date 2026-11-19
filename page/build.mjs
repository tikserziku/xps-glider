// Build ONE self-contained offline HTML: esbuild bundle (three.js from the vendored pilot) + results.json + 10 GLB (base64).
// Usage: node build.mjs [pdfUrl]      -> ..\out\glider.html
// Gate (WEB-VIZ-RULES P4): refuses to build when out\3d\check.json is not ok.
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const out = path.resolve(here, '../out');
// node modules (three, esbuild): `npm install` in this folder, or a vendored copy named by GLIDER_NODE_MODULES_ROOT
const pilot = [here, process.env.GLIDER_NODE_MODULES_ROOT, path.resolve(here, '../../web-drawing-tech/pilot')]
  .find(d => d && fs.existsSync(path.join(d, 'node_modules', 'esbuild')));
if (!pilot) { console.error('esbuild / three not found - run `npm install` in page/'); process.exit(2); }
const { build } = createRequire(path.join(pilot, 'package.json'))('esbuild');

const check = JSON.parse(fs.readFileSync(path.join(out, '3d/check.json'), 'utf8'));
if (!check.ok) { console.error('out/3d/check.json is not ok - refusing to build'); process.exit(3); }

const data = JSON.parse(fs.readFileSync(path.join(out, 'results.json'), 'utf8'));
for (const f of data.forms) delete f.path;                       // the page simulates itself
const glb = {};
for (const f of data.forms) glb[f.id] = fs.readFileSync(path.join(out, '3d', f.id + '.glb')).toString('base64');

const r = await build({ entryPoints: [path.join(here, 'src/app.js')], bundle: true, minify: true, format: 'iife', write: false,
  target: 'es2022', legalComments: 'none', absWorkingDir: here, nodePaths: [path.join(pilot, 'node_modules')] });
const js = r.outputFiles[0].text.replace(/<\/script/gi, '<\\/script');
const safe = o => JSON.stringify(o).replace(/<\//g, '<\\/');
const built = new Date().toISOString().slice(0, 16).replace('T', ' ');
// Usage: node build.mjs [pdfUrl] [chosenFormId] [repoUrl]      chosen = form id built full-size
const cfg = { pdf_url: process.argv[2] || '', chosen: process.argv[3] || '', repo_url: process.argv[4] || '', built };

let html = fs.readFileSync(path.join(here, 'src/template.html'), 'utf8');
html = html.replace('__DATA__', () => safe(data)).replace('__GLB__', () => safe(glb)).replace('__CFG__', () => safe(cfg))
  .replace('__BUILT__', () => built).replace('/*__APP__*/', () => js);
fs.writeFileSync(path.join(out, 'glider.html'), html);
console.log(JSON.stringify({ out: 'out/glider.html', html_kb: Math.round(html.length / 1024), js_kb: Math.round(js.length / 1024),
  glb_b64_kb: Math.round(JSON.stringify(glb).length / 1024), forms: data.forms.length }));
