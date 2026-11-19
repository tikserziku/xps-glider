// QR code matrix for the printed sheets -> out\qr.json {url, size, ec, rows: ["0101...", ...]} (1 = dark module).
// Uses the `qrcode` package from the portable node global prefix. Usage: node make_qr.mjs [url]
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
// `qrcode` package: page/node_modules (npm install in page/) or a global prefix
const globalMods = [path.resolve(here, '../page/node_modules'), process.env.GLIDER_GLOBAL_NODE_MODULES, path.resolve(here, '../../../node/global/node_modules')]
  .find(d => d && fs.existsSync(path.join(d, 'qrcode')));
if (!globalMods) { console.error('qrcode package not found - run `npm install` in page/'); process.exit(2); }
const QRCode = createRequire(path.join(globalMods, 'x.js'))('qrcode');
const url = process.argv[2] || 'https://xps-glider.pages.dev/';
const qr = QRCode.create(url, { errorCorrectionLevel: 'M' });
const n = qr.modules.size, d = qr.modules.data;
const rows = [];
for (let r = 0; r < n; r++) { let s = ''; for (let c = 0; c < n; c++) s += d[r * n + c] ? '1' : '0'; rows.push(s); }
const out = path.resolve(here, '../out/qr.json');
fs.writeFileSync(out, JSON.stringify({ url, size: n, ec: 'M', version: qr.version, rows }));
console.log(JSON.stringify({ out: 'out/qr.json', url, size: n, version: qr.version }));
