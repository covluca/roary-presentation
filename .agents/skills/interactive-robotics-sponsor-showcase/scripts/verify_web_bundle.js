#!/usr/bin/env node
/**
 * verify_web_bundle.js
 *
 * Verifies a multi-script static web showcase (like web/index.html) before browser testing:
 * 1. Parses all <script src="..."> tags in order.
 * 2. Checks that every static document.getElementById('...') in the scripts exists in the HTML.
 * 3. Evaluates all scripts sequentially inside a SINGLE shared vm.Context (mirroring browser
 *    global lexical scope) to catch duplicate `const`/`let` declarations (SyntaxError) or
 *    load-time ReferenceError/TypeError crashes.
 */

'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const htmlArg = process.argv[2] || 'web/index.html';
const htmlPath = path.resolve(process.cwd(), htmlArg);

if (!fs.existsSync(htmlPath)) {
  console.error(`[FAIL] HTML file not found: ${htmlPath}`);
  process.exit(1);
}

const htmlDir = path.dirname(htmlPath);
const html = fs.readFileSync(htmlPath, 'utf8');

// 1. Extract all <script src="..."> in order
const scriptTags = [...html.matchAll(/<script\s+[^>]*src=["']([^"']+)["'][^>]*>/gi)].map(m => m[1]);
if (scriptTags.length === 0) {
  console.error(`[FAIL] No <script src="..."> tags found in ${htmlArg}`);
  process.exit(1);
}

console.log(`[1/3] Found ${scriptTags.length} script(s): ${scriptTags.join(', ')}`);

// 2. Check static getElementById('...') calls against HTML IDs
let missingIds = 0;
for (const relSrc of scriptTags) {
  const fullSrc = path.resolve(htmlDir, relSrc);
  const code = fs.readFileSync(fullSrc, 'utf8');
  const idMatches = [...code.matchAll(/document\.getElementById\(\s*['"]([^'"]+)['"]\s*\)/g)].map(m => m[1]);
  for (const id of new Set(idMatches)) {
    const idRegex = new RegExp(`id=["']${id}["']`);
    if (!idRegex.test(html)) {
      console.error(`[FAIL] Missing DOM element id="${id}" in ${htmlArg} (referenced in ${relSrc})`);
      missingIds++;
    }
  }
}
if (missingIds > 0) {
  process.exit(1);
}
console.log(`[2/3] All static document.getElementById() references exist in ${htmlArg}.`);

// 3. Execute all scripts in a SINGLE shared VM context (catches duplicate top-level const/let)
const mockCtx2d = {
  save: () => {}, restore: () => {}, beginPath: () => {}, closePath: () => {},
  moveTo: () => {}, lineTo: () => {}, stroke: () => {}, fill: () => {},
  fillRect: () => {}, strokeRect: () => {}, clearRect: () => {},
  arc: () => {}, ellipse: () => {}, roundRect: () => {}, rect: () => {},
  createLinearGradient: () => ({ addColorStop: () => {} }),
  createRadialGradient: () => ({ addColorStop: () => {} }),
  fillText: () => {}, measureText: () => ({ width: 48 }),
  translate: () => {}, rotate: () => {}, scale: () => {},
  setLineDash: () => {},
};

const sharedGlobal = {
  console,
  Math,
  Date,
  setTimeout: () => 1,
  clearTimeout: () => {},
  setInterval: () => 1,
  clearInterval: () => {},
  requestAnimationFrame: () => 1,
  performance: { now: () => 1000 },
  window: {
    addEventListener: () => {},
    removeEventListener: () => {},
  },
  document: {
    getElementById: () => ({
      getContext: () => mockCtx2d,
      addEventListener: () => {},
      classList: { add: () => {}, remove: () => {}, contains: () => false },
      getBoundingClientRect: () => ({ left: 0, top: 0, width: 760, height: 580 }),
      setPointerCapture: () => {},
      width: 760,
      height: 580,
      checked: true,
      textContent: '',
      className: '',
      innerHTML: '',
    }),
    addEventListener: () => {},
  },
};
sharedGlobal.window = Object.assign(sharedGlobal.window, sharedGlobal);
sharedGlobal.globalThis = sharedGlobal;

const vmContext = vm.createContext(sharedGlobal);

try {
  for (const relSrc of scriptTags) {
    const fullSrc = path.resolve(htmlDir, relSrc);
    const code = fs.readFileSync(fullSrc, 'utf8');
    vm.runInContext(code, vmContext, { filename: relSrc });
  }
  // Also test one control() and render() tick if defined
  if (typeof vmContext.control === 'function') vmContext.control();
  if (typeof vmContext.render === 'function') vmContext.render();
  console.log(`[3/3] Shared global lexical scope & initial control()+render() tick passed with zero errors!`);
} catch (err) {
  console.error(`[FAIL] Browser script execution error in shared global scope:\n`, err);
  process.exit(1);
}
