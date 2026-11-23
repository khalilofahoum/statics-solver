/* Statics Solver — editor, drawing and calculation report. Analysis lives in solver.js. */
(function () {
  'use strict';
  const $ = s => document.querySelector(s);
  const svg = $('#canvas'), panel = $('#panel');
  const LS = 'statics-solver-model-v1';
  let model, res = null, sel = null, tool = 'select', tab = 'inspect';
  const view = { ox: 140, oy: 380, z: 60 };
  const show = { signs: true, loads: true, react: true, N: false, S: false, M: true, def: false, virt: false, il: true, labels: true, fibre: true };
  const scale = { diag: 1, def: 1 };
  // optional user colours for the positive and negative parts of the N, S, M diagrams (null = the diagram's own colour)
  const dcol = { pos: null, neg: null };
  try { const c = JSON.parse(localStorage.getItem('statics-solver-model-v1-dcol') || '{}'); ['pos', 'neg'].forEach(k => { if (/^#[0-9a-f]{6}$/i.test(c[k] || '')) dcol[k] = c[k]; }); } catch (e) { /* ignore */ }
  let probe = null, mmQ = null, mm = null, ilQ = null, il = null;
  let fmChoice = null, fmCache, fmShow = -1; // force method: chosen redundants, cached result, diagram shown
  let undo = [], redo = [], pending = null, drag = null, spaceDown = false;
  const mouse = { x: 0, y: 0 };

  // ---------- helpers
  const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  let FS = 1, DS = 1; // force and displacement scales of the current solution, to hide round-off
  function fmt(v, p) {
    if (v == null || !isFinite(v)) return '–';
    const a = Math.abs(v);
    if (a < 1e-12) return '0';
    let s = (a >= 1e6 || a < 1e-3) ? v.toExponential(3).replace(/\.?0+e/, 'e').replace('e+', 'e') : String(+v.toPrecision(p || 5));
    return s.replace(/-/g, '−');
  }
  const fF = (v, p) => fmt(Math.abs(v) < 2e-6 * FS ? 0 : v, p), fD = (v, p) => fmt(Math.abs(v) < 2e-6 * DS ? 0 : v, p);
  function scales() {
    FS = 0; DS = 0; if (!res || !res.ok) { FS = DS = 1; return; }
    let Lm = 0; res.members.forEach(mr => { Lm = Math.max(Lm, mr.L); mr.segs.forEach(sg => { [0, sg.L].forEach(x => { FS = Math.max(FS, Math.abs(Solver.pval(sg.N, x)), Math.abs(Solver.pval(sg.S, x)), Math.abs(Solver.pval(sg.M, x)) / mr.L); DS = Math.max(DS, Math.abs(Solver.pval(sg.u, x)), Math.abs(Solver.pval(sg.v, x)), Math.abs(Solver.pval(sg.th, x)) * mr.L); }); }); });
    res.reactions.forEach(R => { FS = Math.max(FS, Math.abs(R.Rx), Math.abs(R.Ry)); });
    FS = FS || 1; DS = DS || 1;
  }
  const f1 = v => (Math.round(v * 10) / 10);
  const U = () => model.settings;
  const uF = () => esc(U().fu || 'kN'), uL = () => esc(U().lu || 'm');

  function blank() {
    return { v: 1, nextId: 1, settings: { grid: 0.5, E: 2e8, A: 0.01, I: 1e-4, axialRigid: false, fu: 'kN', lu: 'm' }, nodes: [], members: [], loads: [] };
  }
  const nid = () => model.nextId++;
  const node = id => model.nodes.find(n => n.id === id);
  const member = id => model.members.find(m => m.id === id);
  const nName = n => { const i = model.nodes.indexOf(n); return String.fromCharCode(65 + i % 26) + (i >= 26 ? Math.floor(i / 26) : ''); };
  const mName = m => String(model.members.indexOf(m) + 1);
  function addNode(x, y) { const n = { id: nid(), x, y, sup: 'none', ang: 0, kx: 0, ky: 0, kr: 0, dx: 0, dy: 0, dr: 0 }; model.nodes.push(n); return n; }
  function addMember(n1, n2, o) {
    const s = U();
    const m = Object.assign({ id: nid(), n1, n2, E: s.E, A: s.A, I: s.I, rel: [0, 0, 0, 0, 0, 0], alpha: 1.2e-5, Tt: 0, Tb: 0, h: 0.3, dL: 0 }, o || {});
    model.members.push(m); return m;
  }
  function addLoad(l) { l.id = nid(); model.loads.push(l); return l; }
  function newDist(shape, mid) {
    const l = { type: 'dist', member: mid, t1: 0, t2: 1, q1: 10, q2: 10, dir: 'Y', shape };
    if (shape === 'parabola') { l.q1 = 0; l.qm = 10; l.q2 = 0; }
    if (shape === 'func') l.expr = '10*sin(pi*x/L)';
    return addLoad(l);
  }
  // intensity of a distributed load at fraction f of its drawn range (from t1 to t2)
  function distAt(l, L) { const span = Math.abs(l.t2 - l.t1) * L; return f => Solver.distQ(l, l.t1 <= l.t2 ? f : 1 - f, span); }
  function mGeo(m) { const a = node(m.n1), b = node(m.n2), dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy) || 1e-9; return { a, b, L, c: dx / L, s: dy / L }; }
  const W = (x, y) => [view.ox + x * view.z, view.oy - y * view.z];
  const toWorld = (sx, sy) => [(sx - view.ox) / view.z, (view.oy - sy) / view.z];
  const snap = v => { const g = U().grid > 0 ? U().grid : 0; return g ? Math.round(v / g) * g : v; };

  function change(fn) {
    undo.push(JSON.stringify(model)); if (undo.length > 150) undo.shift(); redo = [];
    fn(); refresh();
  }
  function restore(json) { model = JSON.parse(json); sel = null; pending = null; refresh(); }

  const targetOK = t => t && (t.node != null ? !!node(t.node) : !!member(t.member));
  function refresh() {
    if (sel && !(sel.kind === 'node' ? node(sel.id) : member(sel.id))) sel = null;
    if (!targetOK(probe)) probe = null;
    if (mmQ && !targetOK(mmQ.target)) mmQ = null;
    if (ilQ && !(ilQ.kind === 'reaction' ? node(ilQ.node) : member(ilQ.member))) ilQ = null;
    if (pending && !node(pending)) pending = null;
    res = Solver.analyze(model); scales(); fmCache = undefined; clCache = undefined;
    mm = mmQ && res.ok ? Solver.unitLoad(model, mmQ.target, mmQ.dir) : null;
    il = ilQ && res.ok ? Solver.influence(model, ilQ, 24) : null;
    const b = $('#badge');
    if (!model.members.length) { b.className = 'badge'; b.textContent = 'Empty model'; }
    else if (!res.ok) { b.className = 'badge bad'; b.textContent = res.unstable ? 'Unstable — mechanism' : 'Cannot solve'; }
    else if (res.det.n === 0) { b.className = 'badge ok'; b.textContent = 'Statically determinate'; }
    else { b.className = 'badge'; b.textContent = 'Statically indeterminate · degree ' + res.det.n; }
    render(); renderPanel(); renderChips();
    try { localStorage.setItem(LS, JSON.stringify(model)); } catch (e) { /* storage unavailable */ }
  }

  // ---------- model editing operations
  function splitMember(m, t, nodeId) {
    const L0 = mGeo(m).L;
    const m2 = Object.assign({}, m, { id: nid(), n1: nodeId, n2: m.n2, rel: [0, 0, 0, m.rel[3], m.rel[4], m.rel[5]] });
    m.n2 = nodeId; m.rel = [m.rel[0], m.rel[1], m.rel[2], 0, 0, 0];
    if (m.dL) { m2.dL = m.dL * (1 - t); m.dL = m.dL * t; }
    model.members.splice(model.members.indexOf(m) + 1, 0, m2);
    const extra = [];
    model.loads.forEach(l => {
      if (l.member !== m.id) return;
      if (l.type === 'point') {
        if (Math.abs(l.t - t) < 1e-9) { delete l.member; delete l.t; l.node = nodeId; }
        else if (l.t > t) { l.member = m2.id; l.t = (l.t - t) / (1 - t); } else l.t = l.t / t;
      } else {
        const a = Math.min(l.t1, l.t2), b = Math.max(l.t1, l.t2), span = (b - a) * L0, qAt = tau => Solver.distQ(l, tau, span);
        const part = (ta, tb) => {
          const o = { q1: qAt(ta), q2: qAt(tb) };
          if (l.shape === 'parabola') o.qm = qAt((ta + tb) / 2);
          if (l.shape === 'func') { o.q1 = o.q2 = 0; o.Lx = +l.Lx > 0 ? +l.Lx : span; o.xo = (+l.xo || 0) + (l.rev ? 1 - tb : ta) * span; }
          return o;
        };
        if (b <= t + 1e-9) Object.assign(l, part(0, 1), { t1: a / t, t2: b / t });
        else if (a >= t - 1e-9) Object.assign(l, part(0, 1), { member: m2.id, t1: (a - t) / (1 - t), t2: (b - t) / (1 - t) });
        else { const tsp = (t - a) / (b - a), p1 = part(0, tsp), p2 = part(tsp, 1); extra.push(Object.assign({}, l, p2, { id: nid(), member: m2.id, t1: 0, t2: (b - t) / (1 - t) })); Object.assign(l, p1, { t1: a / t, t2: 1 }); }
      }
    });
    model.loads.push(...extra);
  }
  // node at a world point; splits a member if the point lies inside it
  function nodeAt(x, y) {
    let n = model.nodes.find(q => Math.hypot(q.x - x, q.y - y) < 1e-6);
    if (n) return n;
    n = addNode(x, y);
    for (const m of model.members.slice()) {
      const g = mGeo(m), px = x - g.a.x, py = y - g.a.y, along = px * g.c + py * g.s, off = Math.abs(-px * g.s + py * g.c);
      if (off < 1e-6 && along > 1e-6 && along < g.L - 1e-6) { splitMember(m, along / g.L, n.id); break; }
    }
    return n;
  }
  function deleteNode(n) {
    const ms = model.members.filter(m => m.n1 === n.id || m.n2 === n.id).map(m => m.id);
    model.members = model.members.filter(m => !ms.includes(m.id));
    model.loads = model.loads.filter(l => l.node !== n.id && !ms.includes(l.member));
    model.nodes = model.nodes.filter(q => q !== n);
  }
  function deleteMember(m) {
    model.members = model.members.filter(q => q !== m);
    model.loads = model.loads.filter(l => l.member !== m.id);
  }
  function toggleHingeAtNode(n) {
    const ends = []; model.members.forEach(m => { if (m.n1 === n.id) ends.push([m, 2]); if (m.n2 === n.id) ends.push([m, 5]); });
    const all = ends.every(e => e[0].rel[e[1]]);
    ends.forEach(e => { e[0].rel[e[1]] = all ? 0 : 1; });
  }

  // ---------- picking
  function pickNode(sx, sy, r) {
    let best = null, bd = r || 10;
    model.nodes.forEach(n => { const p = W(n.x, n.y), d = Math.hypot(p[0] - sx, p[1] - sy); if (d < bd) { bd = d; best = n; } });
    return best;
  }
  function pickMember(sx, sy, r) {
    let best = null, bd = r || 8;
    model.members.forEach(m => {
      const g = mGeo(m), a = W(g.a.x, g.a.y), b = W(g.b.x, g.b.y), dx = b[0] - a[0], dy = b[1] - a[1], l2 = dx * dx + dy * dy || 1;
      const t = Math.max(0, Math.min(1, ((sx - a[0]) * dx + (sy - a[1]) * dy) / l2)), d = Math.hypot(a[0] + t * dx - sx, a[1] + t * dy - sy);
      if (d < bd) { bd = d; best = { m, t, g }; }
    });
    return best;
  }
  // snap a picked station to the grid step measured along the member
  function snapT(pm) {
    const g = U().grid > 0 ? U().grid : 0, L = pm.g.L; let x = pm.t * L;
    if (g) { const xs = Math.round(x / g) * g; if (xs > 1e-9 && xs < L - 1e-9) x = xs; }
    return Math.max(0, Math.min(1, x / L));
  }

  // ---------- drawing primitives
  function arrow(x1, y1, x2, y2, cls, fcls) {
    const a = Math.atan2(y2 - y1, x2 - x1), hh = 8, w = 3.6, bx = x2 - hh * Math.cos(a), by = y2 - hh * Math.sin(a);
    return `<line class="${cls}" x1="${f1(x1)}" y1="${f1(y1)}" x2="${f1(bx)}" y2="${f1(by)}"/><path class="${fcls}" d="M${f1(x2)},${f1(y2)}L${f1(bx - w * Math.sin(a))},${f1(by + w * Math.cos(a))}L${f1(bx + w * Math.sin(a))},${f1(by - w * Math.cos(a))}Z"/>`;
  }
  function momentArc(x, y, ccw, r, cls, fcls) {
    const d2r = Math.PI / 180, ts = (ccw ? 45 : 135) * d2r, te = (ccw ? -225 : 405) * d2r;
    const p = t => [x + r * Math.cos(t), y + r * Math.sin(t)], s = p(ts), e = p(te);
    const tx = ccw ? Math.sin(te) : -Math.sin(te), ty = ccw ? -Math.cos(te) : Math.cos(te);
    const tip = [e[0] + tx * 6, e[1] + ty * 6], b = [e[0] - tx * 3, e[1] - ty * 3];
    return `<path class="${cls}" d="M${f1(s[0])},${f1(s[1])}A${r},${r} 0 1 ${ccw ? 0 : 1} ${f1(e[0])},${f1(e[1])}"/><path class="${fcls}" d="M${f1(tip[0])},${f1(tip[1])}L${f1(b[0] - ty * 4)},${f1(b[1] + tx * 4)}L${f1(b[0] + ty * 4)},${f1(b[1] - tx * 4)}Z"/>`;
  }
  const text = (x, y, s, cls, anchor) => `<text x="${f1(x)}" y="${f1(y)}" class="${cls || ''}" text-anchor="${anchor || 'middle'}">${esc(s)}</text>`;
  function hatch(y0, w) {
    let s = `<line class="sup" x1="${-w}" y1="${y0}" x2="${w}" y2="${y0}"/>`;
    for (let i = -w + 3; i <= w; i += 6) s += `<line class="sup thin" x1="${i}" y1="${y0}" x2="${i - 5}" y2="${y0 + 6}"/>`;
    return s;
  }
  function supSym(n) {
    const p = W(n.x, n.y); let rot = -(n.ang || 0), g = '';
    if (n.sup === 'pin') g = `<path class="sup" d="M0,0L-10,17L10,17Z"/>` + hatch(17, 15);
    else if (n.sup === 'roller') g = `<path class="sup" d="M0,0L-10,15L10,15Z"/><circle class="sup" cx="-5.5" cy="18.5" r="3"/><circle class="sup" cx="5.5" cy="18.5" r="3"/>` + hatch(22, 15);
    else if (n.sup === 'fixed') {
      const ms = model.members.filter(m => m.n1 === n.id || m.n2 === n.id);
      if (ms.length === 1) { const g2 = mGeo(ms[0]), sg = ms[0].n1 === n.id ? 1 : -1; rot = Math.atan2(sg * g2.c, sg * g2.s) * 180 / Math.PI; }
      g = `<line class="sup" x1="-15" y1="0" x2="15" y2="0" style="stroke-width:3"/>` + hatch(0, 15);
    } else if (n.sup === 'guide') g = `<line class="sup" x1="-13" y1="0" x2="13" y2="0" style="stroke-width:3"/><circle class="sup" cx="-7" cy="5.5" r="3"/><circle class="sup" cx="7" cy="5.5" r="3"/>` + hatch(9.5, 15);
    return g ? `<g transform="translate(${f1(p[0])},${f1(p[1])}) rotate(${f1(rot)})">${g}</g>` : '';
  }
  function springSym(n) {
    const p = W(n.x, n.y), rot = n.sup !== 'none' ? -(n.ang || 0) : 0; let s = '';
    const zig = `<path class="spring" d="M0,0L0,6L-5,9L5,14L-5,19L5,24L0,27L0,32"/><line class="sup" x1="-10" y1="32" x2="10" y2="32"/>`;
    if (n.ky > 0 && !Solver.SUP[n.sup]?.[1]) s += `<g transform="translate(${f1(p[0])},${f1(p[1])}) rotate(${rot})">${zig}</g>`;
    if (n.kx > 0 && !Solver.SUP[n.sup]?.[0]) s += `<g transform="translate(${f1(p[0])},${f1(p[1])}) rotate(${rot + 90})">${zig}</g>`;
    if (n.kr > 0 && !Solver.SUP[n.sup]?.[2]) s += `<path class="spring" d="M${f1(p[0] + 4)},${f1(p[1])}a4,4 0 1 1 -8,0a6.5,6.5 0 1 1 13,0a9,9 0 1 1 -18,0"/><line class="sup" x1="${f1(p[0] - 14)}" y1="${f1(p[1] - 6)}" x2="${f1(p[0] - 14)}" y2="${f1(p[1] + 6)}"/>`;
    if (show.labels && !rot) {
      if (n.ky > 0 && !Solver.SUP[n.sup]?.[1]) s += text(p[0] + 12, p[1] + 26, 'ky = ' + fmt(n.ky, 4), 'mlbl', 'start');
      if (n.kx > 0 && !Solver.SUP[n.sup]?.[0]) s += text(p[0] - 16, p[1] - 12, 'kx = ' + fmt(n.kx, 4), 'mlbl', 'middle');
      if (n.kr > 0 && !Solver.SUP[n.sup]?.[2]) s += text(p[0] - 18, p[1] + 22, 'kθ = ' + fmt(n.kr, 4), 'mlbl', 'end');
    }
    return s;
  }

  // ---------- diagrams
  function drawDiagram(R, kind, cls, tcls, amp) {
    let mx = 0; const data = [];
    model.members.forEach(m => {
      const mr = R.members.get(m.id), pts = [];
      mr.segs.forEach(sg => { const n = 18; for (let i = 0; i <= n; i++) { const s = sg.L * i / n; pts.push([sg.xa + s, Solver.pval(sg[kind], s)]); } });
      pts.forEach(p => { mx = Math.max(mx, Math.abs(p[1])); });
      data.push({ m, mr, pts });
    });
    if (mx < 1e-9) return { svg: '', max: 0 };
    const k = amp / mx, sign = kind === 'M' ? -1 : 1, tol = 1e-6 * mx; let out = '';
    data.forEach(({ m, mr, pts }) => {
      const g = mGeo(m), base = x => W(g.a.x + g.c * x, g.a.y + g.s * x);
      const pt = (x, v, extra) => { const b = base(x), o = v * k * sign + (extra || 0); return [b[0] - g.s * o, b[1] - g.c * o]; };
      const b0 = base(0), b1 = base(g.L);
      // filled separately for the positive and the negative part, each with its own (optional) colour and sign mark
      const sty = sg => { const c = sg > 0 ? dcol.pos : dcol.neg; return c ? `fill:${c};stroke:${c};` : ''; };
      const Dp = { 1: '', '-1': '' }, Ln = { 1: '', '-1': '' }, marks = []; let hl = '', run = null;
      const P2 = q => f1(q[0]) + ',' + f1(q[1]);
      const piece = (xa, va, xb, vb, sg) => { const p1 = pt(xa, va), p2 = pt(xb, vb); Dp[sg] += `M${P2(base(xa))}L${P2(p1)}L${P2(p2)}L${P2(base(xb))}Z`; Ln[sg] += `M${P2(p1)}L${P2(p2)}`; };
      const endRun = () => { if (run && Math.abs(run.v) * k > 18) marks.push(run); run = null; };
      pts.forEach((p, i) => {
        const sg = p[1] >= 0 ? 1 : -1, a = i ? pts[i - 1] : null;
        if (Math.abs(p[1]) > tol) { if (!run || run.sg !== sg) { endRun(); run = { sg, x: p[0], v: p[1] }; } else if (Math.abs(p[1]) > Math.abs(run.v)) { run.x = p[0]; run.v = p[1]; } } else endRun();
        if (i % 3 === 0 && Math.abs(p[1]) > tol) hl += `<line class="dgh ${cls}" style="${sty(sg)}" x1="${f1(base(p[0])[0])}" y1="${f1(base(p[0])[1])}" x2="${f1(pt(p[0], p[1])[0])}" y2="${f1(pt(p[0], p[1])[1])}"/>`;
        if (!a || i === pts.length - 1) { if (Math.abs(p[1]) > tol) Ln[sg] += `M${P2(base(p[0]))}L${P2(pt(p[0], p[1]))}`; }
        if (!a) return;
        if (p[0] - a[0] < 1e-12) { if (Math.abs(p[1] - a[1]) > tol) Ln[Math.abs(p[1]) >= Math.abs(a[1]) ? sg : (a[1] >= 0 ? 1 : -1)] += `M${P2(pt(a[0], a[1]))}L${P2(pt(p[0], p[1]))}`; return; }
        if (a[1] * p[1] < 0) { const x0 = a[0] + (p[0] - a[0]) * Math.abs(a[1]) / (Math.abs(a[1]) + Math.abs(p[1])); piece(a[0], a[1], x0, 0, a[1] > 0 ? 1 : -1); piece(x0, 0, p[0], p[1], sg); }
        else piece(a[0], a[1], p[0], p[1], a[1] + p[1] >= 0 ? 1 : -1);
      });
      endRun();
      [1, -1].forEach(sg => { if (Dp[sg]) out += `<path class="dg ${cls}" style="${sty(sg)}stroke:none" d="${Dp[sg]}"/><path class="dgl ${cls}" style="${sty(sg)}fill:none" d="${Ln[sg]}"/>`; });
      out += hl;
      if (show.signs) marks.forEach(mk => { const q = pt(mk.x, mk.v / 2), c = mk.sg > 0 ? dcol.pos : dcol.neg; out += `<text x="${f1(q[0])}" y="${f1(q[1] + 5)}" class="sgn ${tcls}" text-anchor="middle"${c ? ` style="fill:${c}"` : ''}>${mk.sg > 0 ? '+' : '−'}</text>`; });
      // value labels: segment ends, constants once, interior extremes
      const labs = [];
      const many = mr.segs.length > 6;
      mr.segs.forEach((sg, si) => {
        const v0 = Solver.pval(sg[kind], 0), v1 = Solver.pval(sg[kind], sg.L), pv = si ? mr.segs[si - 1] : null, nx = mr.segs[si + 1];
        if (sg[kind].every((c, i) => !i || Math.abs(c) * Math.pow(sg.L, i) < tol)) { labs.push({ x: sg.xa + sg.L / 2, v: v0, c: 1 }); return; }
        if (!many || !pv || Math.abs(v0 - Solver.pval(pv[kind], pv.L)) > tol) labs.push({ x: sg.xa, v: v0 });
        if (kind === 'M') { // extreme where S = 0
          const S = sg.S, a = S[2] || 0, b = S[1] || 0, c = S[0], roots = [];
          if (Math.abs(a) > 1e-12) { const D = b * b - 4 * a * c; if (D >= 0) { roots.push((-b + Math.sqrt(D)) / (2 * a), (-b - Math.sqrt(D)) / (2 * a)); } }
          else if (Math.abs(b) > 1e-12) roots.push(-c / b);
          roots.forEach(r => { if (r > (many ? 0 : 0.02) * sg.L && r <= (many ? 1 : 0.98) * sg.L) labs.push({ x: sg.xa + r, v: Solver.pval(sg.M, r) }); });
        }
        if (!many || !nx || Math.abs(v1 - Solver.pval(nx[kind], 0)) > tol) labs.push({ x: sg.xb, v: v1 });
      });
      let last = null;
      labs.forEach(l => {
        if (Math.abs(l.v) < 2e-3 * mx) return;
        if (last && Math.abs(last.v - l.v) < tol && (Math.abs(last.x - l.x) < 1e-6 || (last.c && l.c))) return;
        last = l;
        const q = pt(l.x, l.v, Math.sign(l.v * sign) * 11 || 11);
        out += text(q[0], q[1] + 4, kind === 'M' ? fmt(Math.abs(l.v), 4) : fmt(l.v, 4), tcls);
      });
    });
    return { svg: out, max: mx };
  }
  function drawDeformed() {
    let mx = 0; const data = [];
    model.members.forEach(m => {
      const mr = res.members.get(m.id), pts = [];
      mr.segs.forEach(sg => { for (let i = 0; i <= 14; i++) { const s = sg.L * i / 14, u = Solver.pval(sg.u, s), v = Solver.pval(sg.v, s); pts.push([sg.xa + s, u * mr.c - v * mr.s, u * mr.s + v * mr.c]); } });
      pts.forEach(p => { mx = Math.max(mx, Math.hypot(p[1], p[2])); }); data.push({ m, pts });
    });
    if (mx < 1e-14) return { svg: '', k: 0 };
    const k = 45 * scale.def / mx / view.z; let out = '';
    data.forEach(({ m, pts }) => {
      const g = mGeo(m);
      out += `<polyline class="def" points="${pts.map(p => { const q = W(g.a.x + g.c * p[0] + p[1] * k, g.a.y + g.s * p[0] + p[2] * k); return f1(q[0]) + ',' + f1(q[1]); }).join(' ')}"/>`;
    });
    return { svg: out, k };
  }
  function drawIL() {
    let mx = 0; il.forEach(e => e.pts.forEach(p => { mx = Math.max(mx, Math.abs(p.val)); }));
    if (mx < 1e-12) return { svg: '', max: 0 };
    const k = 55 * scale.diag / mx; let out = '';
    il.forEach(e => {
      const m = member(e.member), g = mGeo(m), base = t => W(g.a.x + g.c * g.L * t, g.a.y + g.s * g.L * t);
      const b0 = base(0), b1 = base(1); let d = `M${f1(b0[0])},${f1(b0[1])}`;
      e.pts.forEach((p, i) => { const b = base(p.t); d += `L${f1(b[0])},${f1(b[1] - p.val * k)}`; if (i % 2 === 0) out += `<line class="dgh dIL" x1="${f1(b[0])}" y1="${f1(b[1])}" x2="${f1(b[0])}" y2="${f1(b[1] - p.val * k)}"/>`; });
      out += `<path class="dg dIL" d="${d}L${f1(b1[0])},${f1(b1[1])}Z"/>`;
      const labs = [e.pts[0], e.pts[e.pts.length - 1]];
      let hi = e.pts[0], lo = e.pts[0];
      e.pts.forEach((p, i) => { if (p.val > hi.val) hi = p; if (p.val < lo.val) lo = p; if (i && Math.abs(p.t - e.pts[i - 1].t) < 1e-9) labs.push(p, e.pts[i - 1]); });
      labs.push(hi, lo);
      const seen = [];
      labs.forEach(p => {
        if (Math.abs(p.val) < 1e-4 * mx || seen.some(q => Math.abs(q.t - p.t) < 1e-9 && Math.abs(q.val - p.val) < 1e-9)) return; seen.push(p);
        const b = base(p.t); out += text(b[0], b[1] - p.val * k + (p.val > 0 ? -5 : 13), fmt(p.val, 4), 'tIL');
      });
    });
    return { svg: out, max: mx };
  }

  // ---------- main render
  function render() {
    const w = svg.clientWidth || 800, hgt = svg.clientHeight || 600; let s = '';
    let gs = (U().grid > 0 ? U().grid : 1) * view.z; while (gs < 14) gs *= 2;
    s += `<defs><pattern id="grid" width="${gs}" height="${gs}" patternUnits="userSpaceOnUse" x="${f1(view.ox - 1)}" y="${f1(view.oy - 1)}"><rect class="gdot" width="2" height="2"/></pattern></defs>`;
    s += `<rect class="bgr" width="${w}" height="${hgt}"/><rect width="${w}" height="${hgt}" fill="url(#grid)"/>`;
    const ok = res && res.ok, legend = [];
    // diagrams (under the members)
    if (ok) {
      const amp = 55 * scale.diag;
      [['N', 'dN', 'tN', `N [${U().fu}]`], ['S', 'dS', 'tS', `S [${U().fu}]`], ['M', 'dM', 'tM', `M [${U().fu}·${U().lu}] — drawn on the tension side`]].forEach(a => {
        if (!show[a[0]]) return; const r = drawDiagram(res, a[0], a[1], a[2], amp); s += r.svg; legend.push([a[2], a[3] + ' · max |' + a[0] + '| = ' + fmt(r.max, 4)]);
      });
      if (show.virt && mm) { const r = drawDiagram(mm.virt, 'M', 'dV', 'tV', amp); s += r.svg; legend.push(['tV', 'm — virtual moment for the unit load · max = ' + fmt(r.max, 4)]); }
      if (fmShow >= 0 && res.det.n > 0) {
        const F = getFM(), R = F && !F.error ? (fmShow === 0 ? F.real0 : F.virt[fmShow - 1]) : null;
        if (R) { const r = drawDiagram(R, 'M', 'dV', 'tV', amp); s += r.svg; legend.push(['tV', (fmShow === 0 ? 'M₀ — loads on the primary structure' : 'm' + sub(fmShow) + ' — unit redundant X' + sub(fmShow) + ' = 1 on the primary structure') + ' · max = ' + fmt(r.max, 4)]); }
      }
      if (show.il && il) { const r = drawIL(); s += r.svg; legend.push(['tIL', 'Influence line: ' + ilTitle() + ' (unit load ↓ 1, + plotted upward)']); }
    }
    // members
    model.members.forEach(m => {
      const g = mGeo(m), a = W(g.a.x, g.a.y), b = W(g.b.x, g.b.y), on = sel && sel.kind === 'member' && sel.id === m.id;
      if (show.fibre) { const ox = g.s * 5.5, oy = g.c * 5.5, i0 = 7; s += `<line class="fibre" x1="${f1(a[0] + ox + g.c * i0)}" y1="${f1(a[1] + oy - g.s * i0)}" x2="${f1(b[0] + ox - g.c * i0)}" y2="${f1(b[1] + oy + g.s * i0)}"/>`; }
      s += `<line class="mem${on ? ' sel' : ''}" x1="${f1(a[0])}" y1="${f1(a[1])}" x2="${f1(b[0])}" y2="${f1(b[1])}"/>`;
      if (show.labels) s += text((a[0] + b[0]) / 2 - g.s * 13, (a[1] + b[1]) / 2 - g.c * 13 + 4, mName(m), 'mlbl');
    });
    if (ok && show.def) { const r = drawDeformed(); s += r.svg; if (r.k) legend.push(['tV', 'Deformed shape · displacements ×' + fmt(r.k, 3)]); }
    // supports, springs
    model.nodes.forEach(n => { s += supSym(n) + springSym(n); });
    // hinges and other releases
    const endsAt = {};
    model.members.forEach(m => { [[m.n1, 2], [m.n2, 5]].forEach(e => { const o = endsAt[e[0]] || (endsAt[e[0]] = { k: 0, h: 0 }); o.k++; if (m.rel[e[1]]) o.h++; }); });
    model.members.forEach(m => {
      const g = mGeo(m);
      [[g.a, 0, 1], [g.b, 3, -1]].forEach(e => {
        const p = W(e[0].x, e[0].y), o = endsAt[e[0].id], dx = g.c * e[2], dy = -g.s * e[2];
        if (m.rel[e[1] + 2] && !(o.h === o.k)) s += `<circle class="hinge" cx="${f1(p[0] + dx * 10)}" cy="${f1(p[1] + dy * 10)}" r="4.2"/>`;
        const tags = (m.rel[e[1]] ? 'N' : '') + (m.rel[e[1] + 1] ? 'S' : '');
        if (tags) s += `<rect class="hinge" x="${f1(p[0] + dx * 20 - 7)}" y="${f1(p[1] + dy * 20 - 7)}" width="14" height="14" rx="3"/>` + text(p[0] + dx * 20, p[1] + dy * 20 + 4, tags.length > 1 ? '∥' : tags === 'N' ? '↔' : '↕', 'mlbl');
      });
    });
    // nodes
    model.nodes.forEach(n => {
      const p = W(n.x, n.y), on = sel && sel.kind === 'node' && sel.id === n.id, o = endsAt[n.id];
      if (o && o.h === o.k && o.k > 1) s += `<circle class="hinge" cx="${f1(p[0])}" cy="${f1(p[1])}" r="5"/>` + (on ? `<circle class="nd sel" cx="${f1(p[0])}" cy="${f1(p[1])}" r="2.5"/>` : '');
      else s += `<circle class="nd${on ? ' sel' : ''}" cx="${f1(p[0])}" cy="${f1(p[1])}" r="${on ? 5 : 3.6}"/>`;
      if (show.labels) s += text(p[0] + 9, p[1] - 9, nName(n), 'lbl', 'start');
    });
    // loads
    if (show.loads) {
      let qmax = 0; model.loads.forEach(l => { if (l.type !== 'dist') return; const m = member(l.member); if (!m) return; const q = distAt(l, mGeo(m).L); for (let i = 0; i <= 8; i++) qmax = Math.max(qmax, Math.abs(q(i / 8))); });
      model.loads.forEach(l => {
        if (l.type === 'dist') {
          const m = member(l.member); if (!m) return; const g = mGeo(m); let dw;
          switch (l.dir) { case 'X': dw = [1, 0]; break; case 'perp': dw = [g.s, -g.c]; break; case 'axial': dw = [g.c, g.s]; break; default: dw = [0, -1]; }
          const ds = [dw[0], -dw[1]], A = W(g.a.x + g.c * g.L * l.t1, g.a.y + g.s * g.L * l.t1), B = W(g.a.x + g.c * g.L * l.t2, g.a.y + g.s * g.L * l.t2);
          const qAt = distAt(l, g.L), curved = l.shape === 'parabola' || l.shape === 'func', len = Math.hypot(B[0] - A[0], B[1] - A[1]);
          let n = Math.max(curved ? 8 : 2, Math.round(len / (curved ? 16 : 22))); if (curved) n += n % 2;
          const par = Math.abs(ds[0] * (-g.s) + ds[1] * (-g.c)) < 0.2, off = par ? [-g.s * 11, -g.c * 11] : [0, 0];
          const hh = q => Math.abs(q) < 1e-12 ? 0 : (12 + 22 * Math.abs(q) / (qmax || 1)) * Math.sign(q), tails = [];
          for (let i = 0; i <= n; i++) {
            const f = i / n, q = qAt(f), hq = hh(q), bx = A[0] + (B[0] - A[0]) * f + off[0], by = A[1] + (B[1] - A[1]) * f + off[1], tx = bx - ds[0] * hq, ty = by - ds[1] * hq;
            if (Math.abs(hq) > 3) s += arrow(tx, ty, bx, by, 'ld', 'ldf'); tails.push([tx, ty]);
          }
          s += `<polyline class="ld" points="${tails.map(t => f1(t[0]) + ',' + f1(t[1])).join(' ')}"/>`;
          const lab = (t, q, str) => Math.abs(q) < 1e-12 && !str ? '' : text(t[0] - ds[0] * 9 * (Math.sign(q) || 1), t[1] - ds[1] * 9 * (Math.sign(q) || 1) + 4, str || fmt(Math.abs(q), 4), 'ldt');
          if (l.shape === 'func') s += lab(tails[n / 2], qAt(0.5) || 1, 'q(x) = ' + l.expr);
          else if (l.shape === 'parabola') s += lab(tails[0], qAt(0)) + lab(tails[n / 2], qAt(0.5)) + lab(tails[n], qAt(1));
          else if (Math.abs(qAt(0) - qAt(1)) < 1e-9) s += lab(tails[Math.floor(n / 2)], qAt(0)); else s += lab(tails[0], qAt(0)) + lab(tails[n], qAt(1));
        } else {
          let x, y;
          if (l.node != null) { const n = node(l.node); if (!n) return; x = n.x; y = n.y; }
          else { const m = member(l.member); if (!m) return; const g = mGeo(m); x = g.a.x + g.c * g.L * l.t; y = g.a.y + g.s * g.L * l.t; }
          const p = W(x, y), mag = Math.hypot(l.Px || 0, l.Py || 0);
          if (mag > 1e-12) { const dx = (l.Px || 0) / mag, dy = (l.Py || 0) / mag; s += arrow(p[0] - dx * 52, p[1] - dy * 52, p[0] - dx * 5, p[1] - dy * 5, 'ld', 'ldf') + text(p[0] - dx * 62, p[1] - dy * 62 + 4, fmt(mag, 4), 'ldt'); }
          if (Math.abs(l.M || 0) > 1e-12) s += momentArc(p[0], p[1], l.M > 0, 17, 'ld', 'ldf') + text(p[0] + 24, p[1] - 20, fmt(Math.abs(l.M), 4), 'ldt', 'start');
        }
      });
    }
    // reactions
    if (ok && show.react) {
      let fm = 0; res.reactions.forEach(R => { fm = Math.max(fm, Math.abs(R.Rx), Math.abs(R.Ry)); });
      const tol = 1e-7 * (fm || 1);
      res.reactions.forEach(R => {
        const p = W(R.x, R.y), n = node(R.node);
        const seg = (ux, uy, val) => { // drawn on the far side of the node; ux,uy: screen direction of a positive value
          const a = [p[0] - ux * 68, p[1] - uy * 68], b = [p[0] - ux * 32, p[1] - uy * 32];
          return (val > 0 ? arrow(a[0], a[1], b[0], b[1], 'rc', 'rcf') : arrow(b[0], b[1], a[0], a[1], 'rc', 'rcf')) + text(a[0] - ux * 11, a[1] - uy * 11 + 4, fmt(Math.abs(val), 4), 'rct');
        };
        if (R.beta && n && (n.sup === 'roller' || n.sup === 'guide') && !R.spring) { if (Math.abs(R.local[1]) > tol) s += seg(-Math.sin(R.beta), -Math.cos(R.beta), R.local[1]); }
        else { if (Math.abs(R.Rx) > tol) s += seg(1, 0, R.Rx); if (Math.abs(R.Ry) > tol) s += seg(0, -1, R.Ry); }
        if (Math.abs(R.M) > 1e-7 * (fm || 1)) s += momentArc(p[0], p[1], R.M > 0, 25, 'rc', 'rcf') + text(p[0] - 30, p[1] - 26, fmt(Math.abs(R.M), 4), 'rct', 'end');
      });
    }
    if (probe) { const q = probeXY(probe); if (q) { const p = W(q[0], q[1]); s += `<circle class="probe" cx="${f1(p[0])}" cy="${f1(p[1])}" r="6"/>`; } }
    legend.forEach((l, i) => { s += text(12, 20 + i * 17, l[1], 'leg ' + l[0], 'start'); });
    if (res && !ok && model.members.length) s += text(12, 20, res.errors[0] || '', 'leg tM', 'start');
    s += '<g id="overlay"></g>';
    svg.innerHTML = s;
    drawOverlay();
  }
  function probeXY(t) {
    if (t.node != null) { const n = node(t.node); return n ? [n.x, n.y] : null; }
    const m = member(t.member); if (!m) return null; const g = mGeo(m); return [g.a.x + g.c * g.L * t.t, g.a.y + g.s * g.L * t.t];
  }
  function drawOverlay() {
    const o = svg.querySelector('#overlay'); if (!o) return; let s = '';
    if (tool === 'member' && pending) { const n = node(pending), a = W(n.x, n.y), w = toWorld(mouse.x, mouse.y), pn = pickNode(mouse.x, mouse.y), b = pn ? W(pn.x, pn.y) : W(snap(w[0]), snap(w[1])); s += `<line class="rub" x1="${f1(a[0])}" y1="${f1(a[1])}" x2="${f1(b[0])}" y2="${f1(b[1])}"/>`; }
    o.innerHTML = s;
  }

  // ---------- view chips
  function renderChips() {
    const c = $('#chips'); c.innerHTML = '';
    const items = [['loads', 'Loads'], ['react', 'Reactions'], ['N', 'N'], ['S', 'S'], ['M', 'M'], ['def', 'Deformed']];
    if (mm) items.push(['virt', 'Virtual m']); if (il) items.push(['il', 'Influence line']);
    items.push(['signs', '+ / − signs'], ['labels', 'Labels'], ['fibre', 'Dashed fibre']);
    items.forEach(it => c.append(h('span', { class: 'chip' + (show[it[0]] ? ' on' : ''), onclick: () => { show[it[0]] = !show[it[0]]; render(); renderChips(); } }, it[1])));
    const rng = (label, key) => { const i = h('input', { type: 'range', min: '0.2', max: '3', step: '0.1', value: scale[key] }); i.addEventListener('input', () => { scale[key] = +i.value; render(); }); return h('label', {}, label, i); };
    c.append(rng('Diagram scale', 'diag'), rng('Deflection scale', 'def'));
    const saveCol = () => { try { localStorage.setItem(LS + '-dcol', JSON.stringify(dcol)); } catch (e) { /* ignore */ } };
    const colr = (label, key, dflt) => { const i = h('input', { type: 'color', value: dcol[key] || dflt, title: 'Colour of the ' + (key === 'pos' ? 'positive' : 'negative') + ' part of the N, S, M diagrams' }); i.addEventListener('input', () => { dcol[key] = i.value; saveCol(); render(); if (!c.querySelector('[data-colreset]')) renderChips(); }); return h('label', {}, label, i); };
    c.append(colr('+ colour', 'pos', '#c81e3a'), colr('− colour', 'neg', '#2457d6'));
    if (dcol.pos || dcol.neg) c.append(h('span', { class: 'chip', 'data-colreset': '1', title: 'Go back to one colour per diagram', onclick: () => { dcol.pos = dcol.neg = null; saveCol(); render(); renderChips(); } }, 'Reset colours'));
  }

  // ---------- panel building
  function h(tag, attrs, ...kids) {
    const e = document.createElement(tag);
    for (const k in attrs || {}) { const v = attrs[k]; if (k === 'class') e.className = v; else if (k.startsWith('on')) e.addEventListener(k.slice(2), v); else if (k === 'html') e.innerHTML = v; else if (v !== false && v != null) e.setAttribute(k, v === true ? '' : v); }
    kids.flat().forEach(c => { if (c != null && c !== false) e.append(c.nodeType ? c : document.createTextNode(c)); });
    return e;
  }
  function num(label, get, set) {
    const show0 = () => String(+(+get() || 0).toPrecision(10)), i = h('input', { type: 'number', step: 'any', value: show0() });
    i.addEventListener('change', () => { const v = parseFloat(i.value); if (isFinite(v)) change(() => set(v)); else i.value = show0(); });
    i.addEventListener('keydown', e => { if (e.key === 'Enter') i.blur(); e.stopPropagation(); });
    return h('label', { class: 'fld' }, label, i);
  }
  const numK = (label, obj, key) => num(label, () => obj[key], v => { obj[key] = v; });
  function txt(label, get, set) {
    const i = h('input', { type: 'text', value: get() || '', spellcheck: 'false' });
    i.addEventListener('change', () => change(() => set(i.value.trim())));
    i.addEventListener('keydown', e => { if (e.key === 'Enter') i.blur(); e.stopPropagation(); });
    return h('label', { class: 'fld' }, label, i);
  }
  function pick(label, opts, get, set) {
    const s = h('select', {}, opts.map(o => h('option', { value: o[0], selected: o[0] === get() }, o[1])));
    s.addEventListener('change', () => change(() => set(s.value)));
    return h('label', { class: 'fld' }, label, s);
  }
  function check(label, get, set) {
    const i = h('input', { type: 'checkbox' }); i.checked = !!get(); i.addEventListener('change', () => change(() => set(i.checked)));
    return h('label', { class: 'chk' }, i, label);
  }
  const btn = (label, fn, cls) => h('button', { class: 'btn small ' + (cls || ''), onclick: fn }, label);
  const row = (...k) => h('div', { class: 'row' }, k);
  const kv = pairs => h('div', { class: 'kv' }, pairs.flatMap(p => [h('span', {}, p[0]), h('span', {}, p[1])]));

  function loadCard(l, m) {
    const del = h('button', { class: 'x', title: 'Remove this load', onclick: () => change(() => { model.loads = model.loads.filter(q => q !== l); }) }, '×');
    if (l.type === 'dist') {
      const L = mGeo(m).L, shape = l.shape || 'linear', qu = `[${U().fu}/${U().lu}]`;
      const out = [del, h('b', {}, 'Distributed load ', h('span', { class: 'he', dir: 'rtl', lang: 'he' }, 'עומס מפורס')),
        pick('Type', [['linear', 'Linear / uniform — מפורס לינארי'], ['parabola', 'Parabolic — מפורס פרבולי'], ['func', 'Function q(x) — מפורס לפי פונקציה']], () => shape, v => {
          if (v === 'parabola' && shape !== 'parabola') l.qm = shape === 'linear' ? ((+l.q1 || 0) + (+l.q2 || 0)) / 2 : 10;
          if (v === 'func' && !l.expr) l.expr = '10*sin(pi*x/L)';
          if (v !== 'func' && shape === 'func') { l.q1 = l.q1 || 10; l.q2 = l.q2 || 10; }
          l.shape = v;
        }),
        row(num(`from x₁ [${U().lu}]`, () => l.t1 * L, v => { l.t1 = Math.max(0, Math.min(1, v / L)); }), num(`to x₂ [${U().lu}]`, () => l.t2 * L, v => { l.t2 = Math.max(0, Math.min(1, v / L)); }))];
      if (shape === 'linear') out.push(row(numK(`q at x₁ ${qu}`, l, 'q1'), numK(`q at x₂ ${qu}`, l, 'q2')), h('p', { class: 'muted' }, 'Equal values give a uniform load; a zero at one end gives a triangle.'));
      else if (shape === 'parabola') out.push(row(numK(`q at x₁ ${qu}`, l, 'q1'), numK(`q middle ${qu}`, l, 'qm'), numK(`q at x₂ ${qu}`, l, 'q2')),
        h('p', { class: 'muted' }, 'A second-degree parabola through the three values. Full parabola: 0 – q – 0. Half parabola with the vertex at x₁: 0 – q/4 – q; with the vertex at x₂: 0 – 3q/4 – q.'));
      else {
        let bad = ''; try { Solver.compileExpr(l.expr); } catch (err) { bad = err.message; }
        out.push(txt(`q(x) = ${qu}`, () => l.expr, v => { l.expr = v; }), bad ? h('div', { class: 'err' }, 'Cannot read this formula: ' + bad) : null,
          h('p', { class: 'muted' }, 'x is measured from the start of the load and L is its length. Allowed: + − * / ^, brackets, sin cos tan exp ln log sqrt abs, pi, e. Examples: 5 + 2x, 10*(x/L)^3, 8*sin(pi*x/L).'));
      }
      out.push(pick('Direction (positive q)', [['Y', 'Vertical ↓ per member length (self-weight)'], ['proj', 'Vertical ↓ per horizontal projection'], ['X', 'Horizontal →'], ['perp', 'Perpendicular, towards the dashed fibre'], ['axial', 'Along the member (start → end)']], () => l.dir, v => { l.dir = v; }));
      return h('div', { class: 'card' }, out);
    }
    const kids = [del, h('b', {}, l.node != null ? 'Joint load ' : 'Point load on member ', h('span', { class: 'he', dir: 'rtl', lang: 'he' }, 'עומס נקודתי'))];
    if (l.member != null) { const L = mGeo(m).L; kids.push(row(num(`x from start [${U().lu}]`, () => l.t * L, v => { l.t = Math.max(0, Math.min(1, v / L)); }))); }
    kids.push(row(numK(`Px → [${U().fu}]`, l, 'Px'), numK(`Py ↓ [${U().fu}]`, l, 'Py'), numK(`M ↺ [${U().fu}·${U().lu}]`, l, 'M')));
    return h('div', { class: 'card' }, kids);
  }

  function inspectNode(n) {
    const out = [h('h2', {}, 'Node ' + nName(n)), row(numK(`x [${U().lu}]`, n, 'x'), numK(`y [${U().lu}]`, n, 'y'))];
    out.push(h('h3', {}, 'Support'), row(pick('Type', [['none', 'None (free joint)'], ['pin', 'Pin (2 reactions)'], ['roller', 'Roller (1 reaction)'], ['fixed', 'Fixed / clamp (3)'], ['guide', 'Sliding clamp (2)']], () => n.sup, v => { n.sup = v; }),
      n.sup !== 'none' && num('Rotation [°]', () => n.ang, v => { n.ang = v; })));
    const s = Solver.SUP[n.sup];
    if (s) {
      out.push(h('p', { class: 'muted' }, 'Rotation turns the support counter-clockwise. A roller at 0° rolls horizontally; at 90° it rolls vertically.'));
      out.push(h('h3', {}, 'Support settlement'), row(s[0] ? numK(`δx → [${U().lu}]`, n, 'dx') : null, s[1] ? numK(`δy ↑ [${U().lu}]`, n, 'dy') : null, s[2] ? numK('δθ ↺ [rad]', n, 'dr') : null),
        h('p', { class: 'muted' }, 'Along the support axes. A support that sinks has a negative δy.'));
    }
    out.push(h('h3', {}, 'Elastic springs ', h('span', { class: 'he', dir: 'rtl', lang: 'he' }, 'קפיצים')), h('p', { class: 'muted' }, 'kx resists horizontal movement (F = kx·u), ky vertical movement (F = ky·v), kθ rotation (M = kθ·θ). Zero means no spring. A direction already held by the support has no spring field.'), row(!(s && s[0]) ? numK(`kx [${U().fu}/${U().lu}]`, n, 'kx') : null, !(s && s[1]) ? numK(`ky [${U().fu}/${U().lu}]`, n, 'ky') : null, !(s && s[2]) ? numK(`kθ [${U().fu}·${U().lu}/rad]`, n, 'kr') : null));
    out.push(h('h3', {}, 'Loads on this joint'));
    model.loads.filter(l => l.node === n.id).forEach(l => out.push(loadCard(l)));
    out.push(h('div', { class: 'btns' }, btn('+ Force / moment', () => change(() => { addLoad({ type: 'point', node: n.id, Px: 0, Py: 10, M: 0 }); })),
      btn('Hinge all members here', () => change(() => toggleHingeAtNode(n)))));
    if (res && res.ok) {
      const d = res.disp.get(n.id), pairs = [['u →', fD(d.u) + ' ' + U().lu], ['v ↑', fD(d.v) + ' ' + U().lu], ['θ ↺', fmt(d.r) + ' rad']];
      res.reactions.filter(R => R.node === n.id).forEach(R => { const t = R.spring ? 'Spring ' : 'Reaction '; pairs.push([t + 'Rx →', fF(R.Rx) + ' ' + U().fu], [t + 'Ry ↑', fF(R.Ry) + ' ' + U().fu], [t + 'M ↺', fF(R.M) + ' ' + U().fu + '·' + U().lu]); });
      out.push(h('h3', {}, 'Results'), kv(pairs));
    }
    out.push(h('div', { class: 'btns' }, btn('Probe this joint', () => { probe = { node: n.id }; refresh(); }), btn('Delete node', () => change(() => deleteNode(n)))));
    return out;
  }
  function inspectMember(m) {
    const g = mGeo(m), a = nName(g.a), b = nName(g.b);
    const out = [h('h2', {}, `Member ${mName(m)} (${a}–${b})`), kv([['Length', fmt(g.L) + ' ' + U().lu], ['Angle', fmt(Math.atan2(g.s, g.c) * 180 / Math.PI, 4) + '°'], ['EI', fmt(m.E * m.I)], ['EA', U().axialRigid ? '∞ (axially rigid)' : fmt(m.E * m.A)]])];
    out.push(h('h3', {}, 'Section'), row(numK('E', m, 'E'), numK('A', m, 'A'), numK('I', m, 'I')));
    out.push(h('h3', {}, 'End connections'));
    const relRow = (name, o) => h('div', { class: 'card' }, h('b', {}, name),
      check('Hinge — releases the moment (M = 0)', () => m.rel[o + 2], v => { m.rel[o + 2] = v ? 1 : 0; }),
      check('Shear slider — releases S', () => m.rel[o + 1], v => { m.rel[o + 1] = v ? 1 : 0; }),
      check('Axial slider — releases N', () => m.rel[o], v => { m.rel[o] = v ? 1 : 0; }));
    out.push(relRow('Start, at ' + a, 0), relRow('End, at ' + b, 3));
    out.push(h('div', { class: 'btns' }, btn('Truss bar (hinge both ends)', () => change(() => { m.rel = [0, 0, 1, 0, 0, 1]; })), btn('Rigid both ends', () => change(() => { m.rel = [0, 0, 0, 0, 0, 0]; }))));
    out.push(h('h3', {}, 'Loads on this member'));
    model.loads.filter(l => l.member === m.id).forEach(l => out.push(loadCard(l, m)));
    out.push(h('div', { class: 'btns' }, btn('+ Point load', () => change(() => { addLoad({ type: 'point', member: m.id, t: 0.5, Px: 0, Py: 10, M: 0 }); })),
      btn('+ Linear q', () => change(() => { newDist('linear', m.id); })), btn('+ Parabolic q', () => change(() => { newDist('parabola', m.id); })), btn('+ Function q(x)', () => change(() => { newDist('func', m.id); }))));
    out.push(h('h3', {}, 'Temperature and imperfection'), row(numK('α [1/°]', m, 'alpha'), numK(`depth h [${U().lu}]`, m, 'h')),
      row(numK('ΔT top fibre', m, 'Tt'), numK('ΔT dashed fibre', m, 'Tb'), numK(`ΔL lack of fit [${U().lu}]`, m, 'dL')),
      h('p', { class: 'muted' }, 'ΔL > 0 means the bar was made too long. The dashed line marks the bottom fibre; positive M puts it in tension.'));
    if (res && res.ok) {
      const p0 = res.at(m.id, 0, 1), p1 = res.at(m.id, g.L, -1);
      out.push(h('h3', {}, 'End forces'), h('div', { class: 'tw', html: `<table><tr><th></th><th>N</th><th>S</th><th>M</th></tr><tr><td>at ${a}</td><td>${fF(p0.N)}</td><td>${fF(p0.S)}</td><td>${fF(p0.M)}</td></tr><tr><td>at ${b}</td><td>${fF(p1.N)}</td><td>${fF(p1.S)}</td><td>${fF(p1.M)}</td></tr></table>` }));
    }
    out.push(h('div', { class: 'btns' }, btn('Flip direction', () => change(() => {
      const t = m.n1; m.n1 = m.n2; m.n2 = t; m.rel = [m.rel[3], m.rel[4], m.rel[5], m.rel[0], m.rel[1], m.rel[2]]; const tt = m.Tt; m.Tt = m.Tb; m.Tb = tt;
      model.loads.forEach(l => { if (l.member !== m.id) return; if (l.type === 'point') l.t = 1 - l.t; else { const a1 = 1 - l.t2, b1 = 1 - l.t1, q = l.q1; l.t1 = a1; l.t2 = b1; l.q1 = l.q2; l.q2 = q; if (l.shape === 'func') l.rev = !l.rev; } });
    })), btn('Delete member', () => change(() => deleteMember(m)))));
    return out;
  }
  function inspectModel() {
    const s = U(), out = [h('h2', {}, 'Model')];
    if (!model.members.length) out.push(h('p', {}, 'Pick the Member tool and click two points on the grid to draw your first member, or load one of the Examples.'));
    else out.push(h('p', { class: 'muted' }, 'Click a node or a member to edit it. Hover a member to read N, S, M anywhere along it.'));
    out.push(h('h3', {}, 'Settings'), row(numK(`Grid step [${s.lu}]`, s, 'grid'), pick('Force unit', [['kN', 'kN'], ['N', 'N'], ['t', 't']], () => s.fu, v => { s.fu = v; }), pick('Length unit', [['m', 'm'], ['mm', 'mm'], ['cm', 'cm']], () => s.lu, v => { s.lu = v; })));
    out.push(check('Neglect axial deformation (EA → ∞) — bending only, as in most hand solutions', () => s.axialRigid, v => { s.axialRigid = v; }));
    out.push(h('h3', {}, 'Default section for new members'), row(numK('E', s, 'E'), numK('A', s, 'A'), numK('I', s, 'I')),
      h('div', { class: 'btns' }, btn('Apply to all members', () => change(() => { model.members.forEach(m => { m.E = s.E; m.A = s.A; m.I = s.I; }); })), btn('Set E = A = I = 1', () => change(() => { s.E = s.A = s.I = 1; model.members.forEach(m => { m.E = m.A = m.I = 1; }); }))),
      h('p', { class: 'muted' }, 'Use consistent units (for example kN and m: E in kN/m², A in m², I in m⁴). In a statically determinate structure the reactions and the N, S, M diagrams do not depend on E, A, I.'));
    if (res && res.ok && res.reactions.length) out.push(h('h3', {}, 'Reactions'), h('div', { class: 'tw', html: reactionTable() }));
    return out;
  }
  function ilTitle() {
    if (!ilQ) return '';
    if (ilQ.kind === 'reaction') return ({ x: 'Rx', y: 'Ry', m: 'M' })[ilQ.comp] + ' at ' + nName(node(ilQ.node));
    const m = member(ilQ.member); return `${ilQ.what} in member ${mName(m)} at x = ${fmt(ilQ.t * mGeo(m).L, 4)}`;
  }
  function probeBlock() {
    const out = [h('h3', {}, 'Probe')];
    if (!probe) { out.push(h('p', { class: 'muted' }, 'Use the Probe tool and click any point of a member (or a joint) to pin the values there, compute its displacement with the unit-load method, or draw an influence line.')); return out; }
    const isN = probe.node != null; let m, L;
    if (isN) out.push(h('p', {}, h('b', {}, 'Joint ' + nName(node(probe.node)))));
    else { m = member(probe.member); L = mGeo(m).L; out.push(h('p', {}, h('b', {}, 'Member ' + mName(m))), row(num(`x from start [${U().lu}]`, () => probe.t * L, v => { probe.t = Math.max(0, Math.min(1, v / L)); if (mmQ && mmQ.target.member === m.id) mmQ.target.t = probe.t; if (ilQ && ilQ.member === m.id) ilQ.t = probe.t; undo.pop(); }))); }
    if (res && res.ok) {
      if (isN) { const d = res.disp.get(probe.node); out.push(kv([['u →', fD(d.u)], ['v ↑', fD(d.v)], ['θ ↺', fmt(d.r)]])); }
      else {
        const l = res.at(m.id, probe.t * L, -1), r = res.at(m.id, probe.t * L, 1), two = (a, b) => Math.abs(a - b) > 1e-9 * (Math.abs(a) + Math.abs(b) + 1e-9) ? fF(a) + ' | ' + fF(b) + '  (left | right)' : fF(a);
        out.push(kv([['N', two(l.N, r.N)], ['S', two(l.S, r.S)], ['M', two(l.M, r.M)], ['u → (global)', fD(l.ux)], ['v ↑ (global)', fD(l.uy)], ['θ ↺', fmt(l.th)]]));
      }
      const tgt = isN ? { node: probe.node } : { member: probe.member, t: probe.t };
      const mmb = d => btn(({ x: 'Δ →', y: 'Δ ↓', r: 'θ ↺' })[d], () => { mmQ = { target: tgt, dir: d }; show.virt = true; tab = 'calc'; setTab(); refresh(); }, mmQ && mmQ.dir === d ? 'pri' : '');
      out.push(h('p', { class: 'muted' }, 'Displacement here by Maxwell–Mohr (unit load):'), h('div', { class: 'btns' }, mmb('x'), mmb('y'), mmb('r')));
      const ilb = (label, q) => btn(label, () => { ilQ = q; show.il = true; refresh(); });
      const btnsIL = [];
      if (isN) { const s = Solver.SUP[node(probe.node).sup]; if (s) { if (s[0]) btnsIL.push(ilb('Rx', { kind: 'reaction', node: probe.node, comp: 'x' })); if (s[1]) btnsIL.push(ilb('Ry', { kind: 'reaction', node: probe.node, comp: 'y' })); if (s[2]) btnsIL.push(ilb('M', { kind: 'reaction', node: probe.node, comp: 'm' })); } }
      else ['M', 'S', 'N'].forEach(w => btnsIL.push(ilb(w, { kind: 'internal', member: probe.member, t: probe.t, what: w })));
      if (btnsIL.length) out.push(h('p', { class: 'muted' }, 'Influence line (unit load ↓ moving over the structure) for:'), h('div', { class: 'btns' }, btnsIL));
      else out.push(h('p', { class: 'muted' }, 'Probe a support joint to draw the influence line of its reaction.'));
    }
    out.push(h('div', { class: 'btns' }, btn('Clear probe', () => { probe = null; mmQ = null; ilQ = null; refresh(); })));
    return out;
  }

  // ---------- calculation report (formulas are TeX, rendered with KaTeX)
  const R = String.raw;
  const sub = i => String(i).split('').map(d => '₀₁₂₃₄₅₆₇₈₉'[d]).join('');
  function tn(v, p) { const s = fmt(v, p).replace(/−/g, '-'), m = /^(-?[\d.]+)e(-?\d+)$/.exec(s); return m ? `${m[1]}\\times10^{${m[2]}}` : s; }
  const tF = (v, p) => tn(Math.abs(v) < 2e-6 * FS ? 0 : v, p), tD = (v, p) => tn(Math.abs(v) < 2e-6 * DS ? 0 : v, p);
  const TX = (t, d) => `<span class="tex${d ? ' d' : ''}">${esc(t)}</span>`;
  const HD = (k, en, he) => `<h3>${k} · ${en} <span class="he" dir="rtl" lang="he">${he}</span></h3>`;
  function polyTex(p, v) {
    let s = ''; const mx = Math.max(...p.map(Math.abs), 1e-12);
    p.forEach((c, i) => {
      if (Math.abs(c) < Math.max(1e-9 * mx, 2e-6 * FS)) return;
      const a = tn(Math.abs(c), 5), term = i === 0 ? a : (a === '1' ? '' : a + '\\,') + v + (i > 1 ? `^{${i}}` : '');
      s += s ? (c < 0 ? ' - ' : ' + ') + term : (c < 0 ? '-' : '') + term;
    });
    return s || '0';
  }
  function renderTex(root) {
    root.querySelectorAll('.tex').forEach(el => {
      if (!window.katex) { el.classList.add('raw'); return; }
      try { window.katex.render(el.textContent, el, { throwOnError: false, displayMode: el.classList.contains('d') }); } catch (e) { el.classList.add('raw'); }
    });
  }
  const signed = (v, first) => (v < 0 ? (first ? '-' : ' - ') : (first ? '' : ' + ')) + tn(Math.abs(v));
  function reactionTable() {
    let t = `<table><tr><th>Support</th><th>Rx → [${uF()}]</th><th>Ry ↑ [${uF()}]</th><th>M ↺ [${uF()}·${uL()}]</th></tr>`;
    res.reactions.forEach(Rc => { t += `<tr><td>${nName(node(Rc.node))}${Rc.spring ? ' (spring)' : ''}</td><td>${fF(Rc.Rx)}</td><td>${fF(Rc.Ry)}</td><td>${fF(Rc.M)}</td></tr>`; });
    return t + '</table>';
  }
  function getFM() {
    if (fmCache === undefined) fmCache = res && res.ok && res.det.n > 0 && res.det.n <= 10 ? Solver.forceMethod(model, fmChoice, res) : null;
    return fmCache;
  }
  function fmLabel(c) {
    if (c.kind === 'R') return ['Rx', 'Ry', 'M'][c.comp] + ' at support ' + nName(node(c.node));
    const m = member(c.member);
    return c.kind === 'M' ? `M in member ${mName(m)} at joint ${nName(node(c.end ? m.n2 : m.n1))}` : 'N in bar ' + mName(m);
  }
  function dofTex(l) {
    const c = ['u', 'v', '\\theta'][l.comp];
    if (l.node != null) return `${c}_{${nName(node(l.node))}}`;
    const m = member(l.member); return `${c}_{${mName(m)}|${tn(l.t * mGeo(m).L, 3)}}`;
  }

  function detReport() {
    const d = res.det; let s = HD(1, 'Static indeterminacy', 'אי-מסוימות סטטית');
    s += TX(R`n = 3m + r - 3j - c = 3\cdot ${d.m} + ${d.r} - 3\cdot ${d.j} - ${d.c} = ${d.n}`, 1);
    const sup = [], relz = [], ends = {};
    model.members.forEach(m => { [[m.n1, 0], [m.n2, 3]].forEach(e => { const o = ends[e[0]] || (ends[e[0]] = { k: 0, h: 0, o: 0 }); o.k++; if (m.rel[e[1] + 2]) o.h++; o.o += (m.rel[e[1]] ? 1 : 0) + (m.rel[e[1] + 1] ? 1 : 0); }); });
    model.nodes.forEach(n => {
      if (!ends[n.id]) return;
      const q = Solver.SUP[n.sup], k = q ? q[0] + q[1] + q[2] : 0, sp = (n.kx > 0 && !(q && q[0]) ? 1 : 0) + (n.ky > 0 && !(q && q[1]) ? 1 : 0) + (n.kr > 0 && !(q && q[2]) ? 1 : 0);
      if (k) sup.push(`${nName(n)}: ${({ pin: 'pin', roller: 'roller', fixed: 'clamp', guide: 'sliding clamp' })[n.sup]} ${k}`);
      if (sp) sup.push(`${nName(n)}: spring ${sp}`);
      const o = ends[n.id], held = (q && q[2]) || n.kr > 0, c = o.h === o.k && !held ? Math.max(o.k - 1, 0) : o.h;
      if (c) relz.push(`${nName(n)}: hinge on ${o.h} of ${o.k} member end${o.k > 1 ? 's' : ''} → ${c}`);
      if (o.o) relz.push(`${nName(n)}: slider release${o.o > 1 ? 's' : ''} → ${o.o}`);
    });
    s += `<div class="tw"><table><tr><th>Count</th><th>Value</th><th style="text-align:left">Where it comes from</th></tr>
<tr><td>m · members</td><td>${d.m}</td><td class="eq">each member carries 3 unknown internal forces (N, S, M)</td></tr>
<tr><td>j · joints</td><td>${d.j}</td><td class="eq">3 equilibrium equations per joint</td></tr>
<tr><td>r · reactions</td><td>${d.r}</td><td class="eq">${esc(sup.join(' · ')) || 'no supports'}</td></tr>
<tr><td>c · releases</td><td>${d.c}</td><td class="eq">${esc(relz.join(' · ')) || 'none'}</td></tr></table></div>`;
    if (model.members.every(m => m.rel[2] && m.rel[5])) s += `<p>All members are pin-ended bars, so the truss count gives the same result:</p>` + TX(R`n = b + r - 2j = ${d.m} + ${d.r} - 2\cdot ${d.j} = ${d.m + d.r - 2 * d.j}`, 1);
    return s;
  }

  function mmReport() {
    let s = HD(5, 'Unit-load method (Maxwell–Mohr)', 'שיטת עומס היחידה');
    if (!mm) return s + '<p class="muted">Place a probe (Probe tool), then choose Δ →, Δ ↓ or θ ↺ in the Inspector to see the virtual-work breakdown here.</p>';
    const t = mmQ.target, where = t.node != null ? 'joint ' + nName(node(t.node)) : `member ${mName(member(t.member))}, x = ${fmt(t.t * mGeo(member(t.member)).L, 4)}`;
    const what = ({ x: 'Horizontal displacement (unit force →)', y: 'Vertical displacement (unit force ↓)', r: 'Rotation (unit moment ↺)' })[mmQ.dir];
    s += `<p>${what} at ${where}.</p>` + TX(R`1\cdot\Delta = \sum\!\int\!\frac{M\,m}{EI}\,dx + \sum\!\int\!\frac{N\,n}{EA}\,dx + \sum\!\int\! n\,\varepsilon_0\,dx + \sum\!\int\! m\,\kappa_0\,dx + \sum\frac{F\,f}{k} - \sum\tilde R\,\delta`, 1);
    s += `<div class="tw"><table><tr><th>Member</th><th>${TX(R`\int\frac{Mm}{EI}dx`)}</th><th>${TX(R`\int\frac{Nn}{EA}dx`)}</th><th>${TX(R`\int n\,\varepsilon_0\,dx`)}</th><th>${TX(R`\int m\,\kappa_0\,dx`)}</th></tr>`;
    mm.rows.forEach(rw => { s += `<tr><td>${rw.idx + 1}</td><td>${fmt(rw.bend)}</td><td>${fmt(rw.axial)}</td><td>${fmt(rw.tAx)}</td><td>${fmt(rw.tBend)}</td></tr>`; });
    s += `<tr><td><b>Σ</b></td><td>${fmt(mm.tot.bend)}</td><td>${fmt(mm.tot.axial)}</td><td>${fmt(mm.tot.tAx)}</td><td>${fmt(mm.tot.tBend)}</td></tr></table></div>`;
    if (mm.springRows.length) { s += `<div class="tw"><table><tr><th>Spring at</th><th>F (real)</th><th>f (virtual)</th><th>k</th><th>F·f/k</th></tr>`; mm.springRows.forEach(q => { s += `<tr><td>${nName(node(q.node))} ${'xyθ'[q.comp]}</td><td>${fmt(q.F)}</td><td>${fmt(q.f)}</td><td>${fmt(q.k)}</td><td>${fmt(q.c)}</td></tr>`; }); s += '</table></div>'; }
    if (mm.settleRows.length) { s += `<div class="tw"><table><tr><th>Settlement at</th><th>R̃ (virtual)</th><th>δ</th><th>−R̃·δ</th></tr>`; mm.settleRows.forEach(q => { s += `<tr><td>${nName(node(q.node))} ${'xyθ'[q.comp]}</td><td>${fmt(q.R)}</td><td>${fmt(q.d)}</td><td>${fmt(q.c)}</td></tr>`; }); s += '</table></div>'; }
    s += TX(R`\Delta = ${tn(mm.total, 6)} \qquad \text{(stiffness method: } ${tn(mm.direct, 6)}\text{)}`, 1);
    s += '<p class="muted">' + TX(R`\varepsilon_0 = \alpha\,\tfrac{\Delta T_{top}+\Delta T_{bottom}}{2} + \tfrac{\Delta L}{L}`) + ', ' + TX(R`\kappa_0 = \alpha\,\tfrac{\Delta T_{bottom}-\Delta T_{top}}{h}`) + '. A positive result is in the direction of the unit load. Turn on “Virtual m” above the drawing to see the m diagram for Vereshchagin’s multiplication.</p>';
    return s;
  }

  function forceReport() {
    let s = HD(6, 'Force (flexibility) method', 'שיטת הגמישות');
    const n = res.det.n;
    if (n === 0) return s + '<p class="muted">The structure is statically determinate, so there are no redundants. Add a support or remove a hinge to see the force method here.</p>';
    const F = getFM();
    if (!F) return s + '<p class="muted">Shown for up to 10 redundants.</p>';
    s += `<p>Degree of indeterminacy n = ${n}: release ${n} redundant${n > 1 ? 's' : ''} X to get a statically determinate primary structure, then restore compatibility at the releases.</p>`;
    s += '<div class="row noprint">';
    for (let i = 0; i < n; i++) {
      const cur = F.chosen[i] ? F.chosen[i].id : '';
      s += `<label class="fld">X${sub(i + 1)}<select data-fm="${i}">${cur ? '' : '<option value="">—</option>'}${F.candidates.map(c => `<option value="${c.id}"${c.id === cur ? ' selected' : ''}>${esc(fmLabel(c))}</option>`).join('')}</select></label>`;
    }
    s += '</div><div class="btns"><button class="btn small" data-fmauto>Choose automatically</button></div>';
    s += '<p>' + F.chosen.map((c, i) => TX(`X_{${i + 1}}`) + ' = ' + esc(fmLabel(c))).join(' · ') + '</p>';
    if (F.error) return s + `<div class="err">${esc(F.error)}</div>`;
    s += `<label class="fld noprint" style="max-width:260px">Show on the drawing<select data-fmshow><option value="-1">Nothing</option><option value="0"${fmShow === 0 ? ' selected' : ''}>M₀ (loads on primary structure)</option>${F.chosen.map((c, i) => `<option value="${i + 1}"${fmShow === i + 1 ? ' selected' : ''}>m${sub(i + 1)} (X${sub(i + 1)} = 1)</option>`).join('')}</select></label>`;
    s += `<p class="muted">Positive X: a support reaction along → / ↑ / ↺ (support axes), a bending moment that puts the dashed fibre in tension, or tension in a bar.</p>`;
    s += TX(R`\Delta_{i0} = \sum\!\int\!\frac{m_i M_0}{EI}dx + \sum\!\int\!\frac{n_i N_0}{EA}dx + \dots \qquad \delta_{ij} = \sum\!\int\!\frac{m_i m_j}{EI}dx + \sum\!\int\!\frac{n_i n_j}{EA}dx + \sum\frac{f_i f_j}{k}`, 1);
    if (n <= 3) {
      const pairs = []; for (let i = 0; i < n; i++) for (let j = i; j < n; j++) pairs.push([i, j]);
      s += '<div class="tw"><table><tr><th>Member</th>' + F.chosen.map((c, i) => `<th>${TX(`\\Delta_{${i + 1}0}`)}</th>`).join('') + pairs.map(q => `<th>${TX(`\\delta_{${q[0] + 1}${q[1] + 1}}`)}</th>`).join('') + '</tr>';
      model.members.forEach((m, k) => { s += `<tr><td>${k + 1}</td>` + F.chosen.map((c, i) => `<td>${fmt(F.D0rows[i][k])}</td>`).join('') + pairs.map(q => `<td>${fmt(F.deltaRows[q[0]][q[1]][k])}</td>`).join('') + '</tr>'; });
      s += '<tr><td><b>Total</b></td>' + F.chosen.map((c, i) => `<td>${fmt(F.D0[i])}</td>`).join('') + pairs.map(q => `<td>${fmt(F.delta[q[0]][q[1]])}</td>`).join('') + '</tr></table></div>';
      s += '<p class="muted">Rows are the integrals over each member; the total also includes any temperature, spring and settlement terms.</p>';
    }
    s += '<p>Compatibility equations ' + TX(R`[\delta]\{X\} + \{\Delta_0\} = \{\delta_s\}`) + ' (δₛ is the prescribed settlement at a released support, otherwise 0):</p>';
    let eqs = R`\begin{aligned}`;
    for (let i = 0; i < n; i++) eqs += F.delta[i].map((d, j) => signed(d, !j) + `\\,X_{${j + 1}}`).join('') + signed(F.D0[i]) + ' &= ' + tn(F.rhs[i]) + R` \\ `;
    s += '<div class="tw">' + TX(eqs + R`\end{aligned}`, 1) + '</div>';
    s += '<div class="tw"><table><tr><th>Redundant</th><th>Force method</th><th>Stiffness method</th></tr>';
    F.chosen.forEach((c, i) => { s += `<tr><td>${TX(`X_{${i + 1}}`)} = ${esc(fmLabel(c))}</td><td>${fmt(F.X[i], 6)}</td><td>${F.check ? fmt(F.check[i], 6) : '–'}</td></tr>`; });
    s += '</table></div>' + TX('M = M_0 + ' + F.chosen.map((c, i) => `X_{${i + 1}}\\,m_{${i + 1}}`).join(' + '), 1) + '<p class="muted">The same superposition gives N, S and the reactions.</p>';
    return s;
  }

  function stiffReport() {
    let s = HD(7, 'Stiffness (displacement) method', 'שיטת הקשיחות');
    const y = res.sys;
    s += `<p>Kinematic indeterminacy <span class="he" dir="rtl" lang="he">אי-מסוימות קינמטית</span>: ${res.kinJ || 0} unknown joint displacements${(res.kin || 0) > (res.kinJ || 0) ? ` (plus ${res.kin - res.kinJ} at the stations where loads split a member)` : ''}. Every member adds its stiffness matrix (local end displacements u₁ v₁ θ₁ u₂ v₂ θ₂); hinged ends are condensed out and member loads enter through the fixed-end forces f₀.</p>`;
    s += '<div class="tw">' + TX(R`[k] = \begin{bmatrix} \frac{EA}{L} & 0 & 0 & -\frac{EA}{L} & 0 & 0 \\[2pt] 0 & \frac{12EI}{L^3} & \frac{6EI}{L^2} & 0 & -\frac{12EI}{L^3} & \frac{6EI}{L^2} \\[2pt] 0 & \frac{6EI}{L^2} & \frac{4EI}{L} & 0 & -\frac{6EI}{L^2} & \frac{2EI}{L} \\[2pt] -\frac{EA}{L} & 0 & 0 & \frac{EA}{L} & 0 & 0 \\[2pt] 0 & -\frac{12EI}{L^3} & -\frac{6EI}{L^2} & 0 & \frac{12EI}{L^3} & -\frac{6EI}{L^2} \\[2pt] 0 & \frac{6EI}{L^2} & \frac{2EI}{L} & 0 & -\frac{6EI}{L^2} & \frac{4EI}{L} \end{bmatrix}`, 1) + '</div>';
    s += TX(R`[K]\{d\} = \{F\} \qquad\qquad \{f\}_{member} = [k]\{d\} + \{f_0\}`, 1);
    if (!y || !y.d) return s + '<p class="muted">The global system has more than 30 unknowns and is not printed.</p>';
    const nq = y.K.length, kmax = Math.max(...y.K.map(r => Math.max(...r.map(Math.abs))), 1e-30), z = v => Math.abs(v) < 1e-9 * kmax ? 0 : v;
    if (nq <= 7) {
      s += '<div class="tw">' + TX(R`\begin{bmatrix}` + y.K.map(r => r.map(v => tn(z(v), 4)).join(' & ')).join(R` \\ `) + R`\end{bmatrix}\begin{Bmatrix}` + y.labels.map(dofTex).join(R` \\ `) + R`\end{Bmatrix} = \begin{Bmatrix}` + y.F.map(v => tF(v, 4)).join(R` \\ `) + R`\end{Bmatrix}`, 1) + '</div>';
    } else {
      s += '<div class="tw"><table><tr><th>[K]</th>' + y.labels.map(l => `<th>${TX(dofTex(l))}</th>`).join('') + '<th>{F}</th></tr>';
      y.K.forEach((r, i) => { s += `<tr><td>${TX(dofTex(y.labels[i]))}</td>` + r.map(v => `<td>${fmt(z(v), 4)}</td>`).join('') + `<td>${fF(y.F[i], 4)}</td></tr>`; });
      s += '</table></div>';
    }
    s += '<p>Solution:</p><div class="tw"><table><tr><th>Unknown</th><th>Value</th></tr>';
    y.labels.forEach((l, i) => { s += `<tr><td>${TX(dofTex(l))}</td><td>${l.comp === 2 ? fmt(y.d[i]) : fD(y.d[i])}</td></tr>`; });
    s += '</table></div><p class="muted">{F} holds the joint loads plus the equivalent joint loads of member loads and temperature (minus the fixed-end forces). An index like 2|3 is a station on member 2 at x = 3 where a point load or a change of loading splits the member. The end forces from {f} = [k]{d} + {f₀} are the values listed in section 3.'
      + (U().axialRigid ? ' Axial deformation is neglected by a very large EA, which is why some terms of [K] are huge.' : '') + '</p>';
    return s;
  }

  function ilReport() {
    let s = HD(10, 'Influence lines', 'קווי השפעה');
    if (!il) return s + '<p class="muted">Place a probe on a member or a support (Probe tool) and choose M, S, N or a reaction under “Influence line” in the Inspector.</p>';
    s += `<p><b>${esc(ilTitle())}</b>. The ordinate η(x) is the value of that response when a unit load ↓ 1 stands at x. For a load train: ${TX(R`Z = \sum P_i\,\eta_i`)}; for a uniform load q over a length: ${TX(R`Z = q\cdot A_\eta`)}.</p>`;
    s += '<div class="tw"><table><tr><th>Member</th><th>max η</th><th>at x</th><th>min η</th><th>at x</th><th>A⁺</th><th>A⁻</th></tr>';
    let Ap = 0, An = 0;
    il.forEach(en => {
      const m = member(en.member), L = mGeo(m).L; let hi = en.pts[0], lo = en.pts[0], ap = 0, an = 0;
      en.pts.forEach((p, i) => {
        if (p.val > hi.val) hi = p; if (p.val < lo.val) lo = p; if (!i) return;
        const q = en.pts[i - 1], dx = (p.t - q.t) * L; if (dx < 1e-12) return;
        if (q.val * p.val >= 0) { const a = (q.val + p.val) / 2 * dx; if (a > 0) ap += a; else an += a; }
        else { const x0 = dx * Math.abs(q.val) / (Math.abs(q.val) + Math.abs(p.val)), a1 = q.val * x0 / 2, a2 = p.val * (dx - x0) / 2; if (a1 > 0) ap += a1; else an += a1; if (a2 > 0) ap += a2; else an += a2; }
      });
      Ap += ap; An += an;
      s += `<tr><td>${mName(m)}</td><td>${fmt(hi.val, 4)}</td><td>${fmt(hi.t * L, 4)}</td><td>${fmt(lo.val, 4)}</td><td>${fmt(lo.t * L, 4)}</td><td>${fmt(ap, 4)}</td><td>${fmt(an, 4)}</td></tr>`;
    });
    s += `<tr><td><b>Σ</b></td><td colspan="4"></td><td>${fmt(Ap, 4)}</td><td>${fmt(An, 4)}</td></tr></table></div>`;
    s += '<p class="muted">Areas are measured along the member length. For the largest positive effect load only the positive parts with q (q·A⁺); for the largest negative effect load only the negative parts (q·A⁻).</p>';
    il.forEach(en => {
      const m = member(en.member), L = mGeo(m).L; s += `<div class="tw"><table><tr><th>Member ${mName(m)}: x</th>`;
      const pts = en.pts.filter((p, i) => i % 3 === 0 || i === en.pts.length - 1 || (i && Math.abs(p.t - en.pts[i - 1].t) < 1e-9) || (i < en.pts.length - 1 && Math.abs(p.t - en.pts[i + 1].t) < 1e-9));
      pts.forEach(p => { s += `<td>${fmt(p.t * L, 3)}</td>`; }); s += '</tr><tr><th>η</th>'; pts.forEach(p => { s += `<td>${fmt(p.val, 4)}</td>`; }); s += '</tr></table></div>';
    });
    return s;
  }

  // ---------- slope-deflection and moment distribution (classic.js)
  let clCache;
  function getCL() { if (clCache === undefined) clCache = res && res.ok ? Classic.analyze(model, Solver) : null; return clCache; }
  const endTex = (m, e) => `M_{${nName(node(m.n[e]))}${nName(node(m.n[1 - e]))}}`;
  const endTxt = (m, e) => nName(node(m.n[e])) + nName(node(m.n[1 - e]));
  const unkTex = C => C.thetaNodes.map(id => `\\theta_{${nName(node(id))}}`).concat(C.modes.map((md, i) => `\\Delta_{${i + 1}}`));
  function linTex(coef, c0, names, cscale) {
    let s = ''; const mx = Math.max(...coef.map(Math.abs), 1e-30);
    coef.forEach((v, i) => { if (Math.abs(v) < 1e-9 * mx) return; s += signed(v, !s) + '\\,' + names[i]; });
    if (Math.abs(c0) > 1e-9 * Math.max(cscale || 0, 1e-12)) s += signed(c0, !s);
    return s || '0';
  }
  const classicNA = C => `<p class="muted">Not set up for this structure: ${esc(C.reasons.join('; '))}. The two hand methods here cover beams and frames under loads, with bending deformation only.</p>`;
  const zc = (v, C) => Math.abs(v) < 1e-9 * Math.max(C.scale, 1e-12) ? 0 : v;

  function slopeReport() {
    let s = HD(8, 'Slope-deflection method', 'שיטת שיפוע-סטייה'); const C = getCL();
    if (!C) return s; if (!C.ok) return s + classicNA(C);
    const names = unkTex(C), dn = names.slice(C.nt);
    s += `<p>The unknowns are the joint rotations θ and the independent joint translations Δ (sway); axial deformation is neglected. An end moment ${TX('M_{ij}')} acts on the member end at joint i. End moments, θ and the chord rotation ψ are positive counter-clockwise ↺, so in the dashed-fibre convention of section 3 the bending moment is ${TX('-M_{ij}')} at the start of a member and ${TX('+M_{ji}')} at its end.</p>`;
    s += '<div class="tw">' + TX(R`M_{ij} = \frac{2EI}{L}\left(2\theta_i + \theta_j - 3\psi_{ij}\right) + M^F_{ij} \qquad\quad \text{far end hinged:}\;\; M_{ij} = \frac{3EI}{L}\left(\theta_i - \psi_{ij}\right) + M^{F\prime}_{ij}`, 1) + '</div>';
    C.stubs.forEach(st => { const m = member(st.member); s += `<p>Member ${mName(m)} is a cantilever, so it is statically determinate: it is replaced by its forces on joint ${nName(node(st.root))} (moment ${fmt(zc(-st.M, C))} ↺, Fx ${fmt(zc(st.Fx, C))} →, Fy ${fmt(zc(st.Fy, C))} ↑).</p>`; });
    const unk = C.thetaNodes.map(id => TX(`\\theta_{${nName(node(id))}}`)).concat(C.modes.map((md, i) => TX(`\\Delta_{${i + 1}}`) + ` = ${md.dir === 'u' ? 'horizontal → ' : 'vertical ↑ '}displacement of joint ${nName(node(md.node))}`));
    s += unk.length ? `<p>Unknowns (${unk.length}): ${unk.join(' · ')}.</p>` : '<p>There is no unknown joint rotation or sway here: every end moment equals its fixed-end moment.</p>';
    s += `<div class="tw"><table><tr><th>Member</th><th style="text-align:left">Ends</th><th>EI/L</th><th>${TX('\\psi')}</th><th>${TX('M^F_{ij}')}</th><th>${TX('M^F_{ji}')}</th></tr>`;
    C.members.forEach(m => {
      const a = nName(node(m.n[0])), b = nName(node(m.n[1])), hd = e => `hinged at ${e ? b : a}${m.eff[e] ? ' (end support)' : ''}`;
      const cond = m.h[0] && m.h[1] ? 'hinged both ends — no end moments' : m.h[0] ? `${hd(0)} – rigid` : m.h[1] ? `rigid – ${hd(1)}` : 'rigid – rigid';
      s += `<tr><td>${mName(member(m.id))} (${a}–${b})</td><td class="eq">${cond}</td><td>${fmt(m.k)}</td><td>${C.modes.length ? TX(linTex(m.psi, 0, dn)) : '0'}</td><td>${fmt(zc(m.F[0], C))}</td><td>${fmt(zc(m.F[1], C))}</td></tr>`;
    });
    s += '</table></div>';
    let eq = ''; C.members.forEach(m => { [0, 1].forEach(e => { if (!m.h[e]) eq += `${endTex(m, e)} &= ${linTex(m.ex[e].coef, m.ex[e].c0, names, C.scale)} ` + R`\\ `; }); });
    if (eq) s += '<p>End moments in terms of the unknowns:</p><div class="tw">' + TX(R`\begin{aligned}` + eq + R`\end{aligned}`, 1) + '</div>';
    if (C.eqs.length) {
      let es = '';
      C.eqs.forEach(q => {
        if (q.kind === 'joint') es += R`\text{Joint ${nName(node(q.node))}:}\quad ` + q.ends.map(en => endTex(C.members.find(m => m.id === en.member), en.e)).join(' + ') + ` = ${tn(zc(q.rhs, C))} &\\;\\Rightarrow\\; ${linTex(q.row, q.c0, names, C.scale)} = ${tn(zc(q.rhs, C))} ` + R`\\ `;
        else es += R`\text{Sway }\Delta_{${q.mode + 1}}:\quad \sum (M_{ij}+M_{ji})\,\psi_{ij} + W_{${q.mode + 1}} = 0 &\;\Rightarrow\; ` + `${linTex(q.row, q.c0, names, C.scale)} = 0 ` + R`\\ `;
      });
      s += '<p>Equilibrium equations — moments at each joint, and virtual work for each sway (W is the work of the loads when Δ = 1 with all member ends hinged):</p><div class="tw">' + TX(R`\begin{aligned}` + es + R`\end{aligned}`, 1) + '</div>';
      if (C.modes.length) s += '<p class="muted">' + C.W.map((w, i) => TX(`W_{${i + 1}} = ${tn(w)}`)).join(' · ') + '</p>';
      s += '<div class="tw"><table><tr><th>Unknown</th><th>Value</th></tr>' + names.map((nm, i) => `<tr><td>${TX(nm)}</td><td>${fmt(C.q[i])}</td></tr>`).join('') + '</table></div>';
    }
    s += '<div class="tw"><table><tr><th>End moment ↺</th>' + C.members.map(m => [0, 1].map(e => `<th>${TX(endTex(m, e))}</th>`).join('')).join('') + '</tr><tr><td>value</td>' + C.members.map(m => [0, 1].map(e => `<td>${fmt(zc(m.Msd[e], C), 5)}</td>`).join('')).join('') + '</tr></table></div>';
    return s;
  }

  function mdTable(C, cs) {
    const E = C.md.ends, ni = id => model.nodes.indexOf(node(id)), ord = E.map((_, k) => k).sort((a, b) => (ni(E[a].node) - ni(E[b].node)) || a - b);
    const cell = v => `<td>${fmt(zc(v, C), 4)}</td>`, SHOW = 4;
    let s = '<div class="tw"><table><tr><th>Joint</th>' + ord.map(k => `<th>${nName(node(E[k].node))}</th>`).join('') + '</tr><tr><th>End</th>' + ord.map(k => `<th>${endTxt(C.members[E[k].mi], E[k].e)}</th>`).join('') + '</tr>';
    s += '<tr><td>DF</td>' + ord.map(k => `<td>${E[k].joint ? fmt(E[k].DF, 4) : '—'}</td>`).join('') + '</tr>';
    const shown = new Array(E.length).fill(0), nrows = Math.min(cs.rows.length, 1 + 2 * SHOW);
    for (let r = 0; r < nrows; r++) { const rw = cs.rows[r], cyc = Math.ceil(r / 2); s += `<tr><td>${rw.type === 'FEM' ? 'Fixed-end M' : rw.type === 'Bal' ? 'Balance ' + cyc : 'Carry-over ' + cyc}</td>` + ord.map(k => cell(rw.vals[k])).join('') + '</tr>'; rw.vals.forEach((v, k) => { shown[k] += v; }); }
    if (cs.rows.length > nrows) s += `<tr><td>Cycles ${SHOW + 1}–${cs.cycles}</td>` + ord.map(k => cell(cs.total[k] - shown[k])).join('') + '</tr>';
    s += '<tr><td><b>Σ</b></td>' + ord.map(k => `<td><b>${fmt(zc(cs.total[k], C), 5)}</b></td>`).join('') + '</tr></table></div>';
    return s;
  }
  function crossReport() {
    let s = HD(9, 'Moment distribution (Cross)', 'שיטת התחלקות המומנטים'); const C = getCL();
    if (!C) return s; if (!C.ok) return s + classicNA(C);
    const D = C.md, nS = C.modes.length;
    s += '<p>Each joint that can rotate is locked, then released in turn: its unbalanced moment is shared between the member ends in proportion to their stiffness, and half of each share is carried over to a rigid far end. Same sign convention as section 8.</p>';
    s += '<div class="tw">' + TX(R`S_{ij} = \frac{4EI}{L}\ \text{(far end rigid)}, \quad S_{ij} = \frac{3EI}{L}\ \text{(far end hinged, no carry-over)}, \qquad DF_{ij} = \frac{S_{ij}}{\sum_j S_{ij}}, \qquad \text{carry-over} = \tfrac12`, 1) + '</div>';
    if (!C.nt) s += '<p class="muted">No joint is free to rotate against another member here, so there is nothing to distribute: the fixed-end moments are final.</p>';
    s += `<p><b>${nS ? 'Case 0 — loads, with the joints held against translation' : 'Distribution'}</b>${D.case0.cycles ? ` (converged after ${D.case0.cycles} cycle${D.case0.cycles > 1 ? 's' : ''})` : ''}</p>` + mdTable(C, D.case0);
    if (nS) {
      C.modes.forEach((md, d) => {
        s += `<p><b>Case ${d + 1} — sway ${TX(`\\Delta_{${d + 1}} = 1`)}</b> (${md.dir === 'u' ? 'horizontal' : 'vertical'} displacement of joint ${nName(node(md.node))}) with the joints not rotating: ${TX(R`M^F = -\frac{6EI}{L}\psi`)} at both ends, or ${TX(R`-\frac{3EI}{L}\psi`)} when the far end is hinged.</p>` + mdTable(C, D.sway[d]);
      });
      let es = '';
      for (let d = 0; d < nS; d++) es += D.Rmat[d].map((v, e) => signed(v, !e) + `\\,\\Delta_{${e + 1}}`).join('') + signed(D.R0[d] + C.W[d]) + ' &= 0 ' + R`\\ `;
      s += '<p>Sway equations ' + TX(R`\sum (M_{ij}+M_{ji})\,\psi_{ij} + W = 0`) + ' written with the totals of the cases:</p><div class="tw">' + TX(R`\begin{aligned}` + es + R`\end{aligned}`, 1) + '</div>';
      s += '<p>' + D.c.map((v, d) => TX(`\\Delta_{${d + 1}} = ${tn(v)}`)).join(' · ') + '</p>' + TX('M = M^{(0)} + ' + D.c.map((v, d) => `\\Delta_{${d + 1}}\\,M^{(${d + 1})}`).join(' + '), 1);
    }
    s += '<p>Final end moments (↺ on the member end) from the three solutions:</p><div class="tw"><table><tr><th>End</th><th>Slope-deflection</th><th>Moment distribution</th><th>Matrix solver, EA → ∞</th></tr>';
    C.members.forEach(m => { [0, 1].forEach(e => { s += `<tr><td>${TX(endTex(m, e))}</td><td>${fmt(zc(m.Msd[e], C), 5)}</td><td>${fmt(zc(m.Mmd[e], C), 5)}</td><td>${fmt(zc(m.Mref[e], C), 5)}</td></tr>`; }); });
    s += '</table></div>';
    if (!U().axialRigid) s += '<p class="muted">Sections 2–3 and the drawing include axial deformation, so they can differ slightly from these hand methods. Tick “Neglect axial deformation” in the Inspector to make them agree exactly.</p>';
    return s;
  }

  function report() {
    if (!model.members.length) return '<p>Draw a structure to see the calculations.</p>';
    const d = res.det; let s = '<h2>Calculations</h2>' + detReport();
    if (!res.ok) return s + res.errors.map(e => `<div class="err">${esc(e)}</div>`).join('');
    s += `<p>${d.n === 0 ? 'Statically determinate <span class="he" dir="rtl" lang="he">מסוים סטטית</span>: equilibrium alone gives every reaction and internal force.' : `Statically indeterminate <span class="he" dir="rtl" lang="he">לא מסוים סטטית</span> to degree ${d.n}: equilibrium is not enough, so the results depend on E, A, I. Sections 6 and 7 solve it with the force and the stiffness method.`}</p>`;
    res.warnings.forEach(w => { s += `<div class="warn">${esc(w)}</div>`; });
    const e = res.equilibrium, a = e.applied, r = e.react;
    s += HD(2, 'Reactions and equilibrium check', 'ריאקציות ושיווי משקל') + `<div class="tw">${reactionTable()}</div>`;
    s += TX(R`\begin{aligned} \sum F_x &= ${tF(a.Fx)} ${signed(Math.abs(r.Fx) < 2e-6 * FS ? 0 : r.Fx)} = ${tF(a.Fx + r.Fx)} \\ \sum F_y &= ${tF(a.Fy)} ${signed(Math.abs(r.Fy) < 2e-6 * FS ? 0 : r.Fy)} = ${tF(a.Fy + r.Fy)} \\ \sum M_{(0,0)} &= ${tF(a.Mo)} ${signed(Math.abs(r.Mo) < 2e-6 * FS ? 0 : r.Mo)} = ${tF(a.Mo + r.Mo)} \end{aligned}`, 1);
    s += '<p class="muted">Each line is (applied loads) + (reactions), with ↑ and ↺ positive.</p>';
    s += HD(3, 'Internal forces, member by member', 'כוחות פנימיים') + '<p class="muted">x runs from the start joint. ' + TX('N>0') + ' tension; ' + TX(R`S = \tfrac{dM}{dx}`) + ', ' + TX(R`q = -\tfrac{dS}{dx}`) + '; ' + TX('M>0') + ' puts the dashed fibre in tension.</p>';
    model.members.forEach(m => {
      const g = mGeo(m), mr = res.members.get(m.id);
      s += `<p><b>Member ${mName(m)} (${nName(g.a)}–${nName(g.b)})</b>, L = ${fmt(g.L)} ${uL()}</p><div class="tw"><table><tr><th>x range</th><th style="text-align:left">Equations</th></tr>`;
      let best = null, shown = 0;
      mr.segs.forEach(sg => {
        const sh = k => polyTex(Solver.pshift(sg[k], sg.xa), 'x');
        if (mr.segs.length <= 14 || shown < 14) { shown++; s += `<tr><td>${fmt(sg.xa, 4)} … ${fmt(sg.xb, 4)}</td><td class="eq">${TX(R`\begin{aligned} N &= ${sh('N')} \\ S &= ${sh('S')} \\ M &= ${sh('M')} \end{aligned}`, 1)}</td></tr>`; }
        for (let i = 0; i <= 40; i++) { const x = sg.L * i / 40, v = Solver.pval(sg.M, x); if (!best || Math.abs(v) > Math.abs(best.v)) best = { v, x: sg.xa + x }; }
      });
      const p0 = res.at(m.id, 0, 1), p1 = res.at(m.id, g.L, -1);
      s += `</table></div><div class="tw"><table><tr><th></th><th>N</th><th>S</th><th>M</th></tr><tr><td>start ${nName(g.a)}</td><td>${fF(p0.N)}</td><td>${fF(p0.S)}</td><td>${fF(p0.M)}</td></tr><tr><td>end ${nName(g.b)}</td><td>${fF(p1.N)}</td><td>${fF(p1.S)}</td><td>${fF(p1.M)}</td></tr><tr><td>largest |M|</td><td colspan="3">${fF(best.v)} at x ≈ ${fmt(best.x, 4)}</td></tr></table></div>`;
    });
    if (model.loads.some(l => l.shape === 'func')) s += '<p class="muted">Under a function load q(x) the member is cut into 12 pieces and each piece is fitted with a cubic, so its equations are listed piece by piece.</p>';
    s += HD(4, 'Joint displacements', 'הזזות') + `<div class="tw"><table><tr><th>Joint</th><th>u → [${uL()}]</th><th>v ↑ [${uL()}]</th><th>θ ↺ [rad]</th></tr>`;
    model.nodes.forEach(n => { const q = res.disp.get(n.id); if (q) s += `<tr><td>${nName(n)}</td><td>${fD(q.u)}</td><td>${fD(q.v)}</td><td>${fmt(Math.abs(q.r) < 2e-8 * DS ? 0 : q.r)}</td></tr>`; });
    s += '</table></div>';
    if (U().axialRigid) s += '<p class="muted">Axial deformation is neglected (EA → ∞).</p>';
    return s + mmReport() + forceReport() + stiffReport() + slopeReport() + crossReport() + ilReport();
  }
  const HELP = `<h2>How to use</h2>
<h3>Build</h3><ul class="help"><li><b>Member</b>: click point after point to draw a chain; Esc or right-click ends it. Clicking on an existing member splits it there.</li>
<li><b>Pin / Roller / Fixed / Slide</b>: click a joint (or a point on a member) to place the support. Rotate it in the Inspector, e.g. an inclined roller.</li>
<li><b>Springs</b>: Vertical, Horizontal and Rotational spring tools — click a joint (or a point on a member) to add one, click again to remove it, and set the stiffness k in the Inspector. Spring forces are shown with the reactions.</li>
<li><b>Hinge</b>: click a joint to hinge all members there, or click near one end of a member to hinge just that end.</li>
<li><b>Loads</b>: Point force and Moment go on a joint or anywhere on a member. Linear q (uniform, trapezoid, triangle), Parabolic q and Function q(x) go on a member. Click, then type the values in the Inspector; every load can cover only part of the member.</li>
<li><b>Select</b>: click to edit, drag a joint to move it, Delete removes it.</li><li><b>Moving the view</b>: the Pan tool, dragging empty space in Select, the middle or right mouse button, holding Space, or the arrow keys. The wheel and + / − zoom; Fit shows everything.</li></ul>
<h3>Results</h3><ul class="help"><li>The structure is re-solved after every change. The badge at the top shows whether it is determinate, indeterminate or a mechanism.</li>
<li>Switch the N, S, M, reactions and deformed-shape layers with the chips above the drawing. Hover any member to read the values at that point.</li>
<li><b>Probe</b> pins a point: from there you get displacements by the unit-load (Maxwell–Mohr) method with the virtual m diagram, and influence lines.</li>
<li>The <b>Calculations</b> tab follows the course: static indeterminacy, reactions, N(x), S(x), M(x), displacements, the unit-load (Maxwell–Mohr) method, the force (flexibility) method, the stiffness method, slope-deflection, moment distribution and influence lines.</li></ul>
<h3>Sign convention (course)</h3><ul class="help"><li>N &gt; 0 is tension. S = dM/dx and q = −dS/dx.</li><li>M &gt; 0 puts the dashed fibre in tension; the M diagram is drawn on the tension side.</li>
<li>Loads: Px → positive, Py ↓ positive, M ↺ positive, q ↓ positive. Reactions and displacements: → and ↑ positive.</li></ul>
<h3>What is modelled</h3><ul class="help"><li>Linear elastic plane beams, frames and trusses with small displacements (Lecture 1 assumptions). Shear deformation is neglected.</li>
<li>Temperature (uniform and gradient), lack of fit, support settlements and elastic springs.</li><li>Slope-deflection and moment distribution are set up for beams and frames under loads (no springs, settlements, temperature or sliders). Plastic analysis is not included.</li></ul>`;

  function renderPanel() {
    panel.innerHTML = '';
    if (tab === 'help') { panel.innerHTML = HELP; return; }
    if (tab === 'calc') {
      panel.innerHTML = report(); renderTex(panel);
      panel.querySelectorAll('select[data-fm]').forEach(el => el.addEventListener('change', () => {
        const F = getFM(), ids = F.chosen.map(c => c.id); while (ids.length < F.n) ids.push('');
        ids[+el.dataset.fm] = el.value; fmChoice = ids; fmCache = undefined; render(); renderPanel();
      }));
      const au = panel.querySelector('[data-fmauto]'); if (au) au.addEventListener('click', () => { fmChoice = null; fmCache = undefined; render(); renderPanel(); });
      const sh = panel.querySelector('select[data-fmshow]'); if (sh) sh.addEventListener('change', () => { fmShow = +sh.value; render(); });
      return;
    }
    let kids;
    if (sel && sel.kind === 'node') kids = inspectNode(node(sel.id));
    else if (sel && sel.kind === 'member') kids = inspectMember(member(sel.id));
    else kids = inspectModel();
    if (res && !res.ok && model.members.length) kids.unshift(...res.errors.map(e => h('div', { class: 'err' }, e)));
    if (res && res.ok) kids.unshift(...res.warnings.map(e => h('div', { class: 'warn' }, e)));
    panel.append(...kids.filter(Boolean), ...probeBlock());
  }
  function setTab() { document.querySelectorAll('.tabs button').forEach(b => b.classList.toggle('on', b.dataset.tab === tab)); }
  document.querySelectorAll('.tabs button').forEach(b => b.addEventListener('click', () => { tab = b.dataset.tab; setTab(); renderPanel(); }));

  // ---------- tools
  const I = d => `<svg viewBox="0 0 20 20">${d}</svg>`;
  const TOOLS = [
    ['', 'Edit'], ['select', 'Select', 'V', I('<path d="M5 3l9 8-4.5.6L12 17l-2 .9-2.4-5.3L5 15z"/>'), 'Click to select and edit · drag a joint to move it · drag empty space to pan'],
    ['erase', 'Erase', 'E', I('<path d="M5 5l10 10M15 5L5 15"/>'), 'Click a joint or member to delete it'],
    ['pan', 'Pan', 'Space', I('<path d="M10 2v16M2 10h16M10 2l-2.5 2.5M10 2l2.5 2.5M10 18l-2.5-2.5M10 18l2.5-2.5M2 10l2.5-2.5M2 10l2.5 2.5M18 10l-2.5-2.5M18 10l-2.5 2.5"/>'), 'Drag to move the drawing · wheel zooms · arrow keys also pan · holding Space pans in any tool'],
    ['', 'Structure'], ['member', 'Member', 'M', I('<path d="M3 16L17 4"/><circle cx="3" cy="16" r="1.6"/><circle cx="17" cy="4" r="1.6"/>'), 'Click the start point, then the end point · keep clicking to chain · Esc to finish'],
    ['node', 'Node', 'N', I('<circle cx="10" cy="10" r="3"/>'), 'Click to place a joint'],
    ['hinge', 'Hinge', 'H', I('<path d="M2 10h5M13 10h5"/><circle cx="10" cy="10" r="3"/>'), 'Click a joint to hinge all members there · click near a member end to hinge only that end'],
    ['', 'Supports'], ['pin', 'Pin', '', I('<path d="M10 4l-5 9h10zM3 16h14"/>'), 'Click a joint or a point on a member to place a pinned support'],
    ['roller', 'Roller', '', I('<path d="M10 3l-5 8h10zM3 17h14"/><circle cx="7" cy="14" r="1.5"/><circle cx="13" cy="14" r="1.5"/>'), 'Click a joint or a point on a member to place a roller'],
    ['fixed', 'Fixed', '', I('<path d="M5 3v14M5 10h12M5 5l-3 3M5 9l-3 3M5 13l-3 3"/>'), 'Click a joint to clamp it'],
    ['guide', 'Slide', '', I('<path d="M3 7h14M3 16h14"/><circle cx="7" cy="11.5" r="2"/><circle cx="13" cy="11.5" r="2"/>'), 'Sliding clamp: holds rotation and one translation'],
    ['', 'Springs'], ['spy', 'Vertical', '', I('<path d="M10 2v2.5l-4 1.5 8 3-8 3 8 3-4 1.5V18M5 18h10"/>'), 'Vertical spring (stiffness ky) — click a joint or a point on a member; click again to remove'],
    ['spx', 'Horizontal', '', I('<path d="M18 10h-2.5l-1.5-4-3 8-3-8-3 8-1.5-4H2M2 5v10"/>'), 'Horizontal spring (stiffness kx) — click a joint or a point on a member; click again to remove'],
    ['spr', 'Rotational', '', I('<path d="M11.5 10a1.5 1.5 0 1 1-3 0 3.5 3.5 0 1 1 7 0 5.5 5.5 0 1 1-11 0 7 7 0 0 1 7-7"/>'), 'Rotational (moment) spring (stiffness kθ) — click a joint; click again to remove'],
    ['', 'Loads'], ['force', 'Point force', 'F', I('<path d="M10 3v13M6 12l4 4 4-4"/>'), 'Concentrated force — click a joint or a point on a member, then edit it in the Inspector'],
    ['moment', 'Moment', '', I('<path d="M15 12a5.5 5.5 0 1 1-1-6"/><path d="M11 5.5l3.2.6.4-3.2"/>'), 'Click a joint or a point on a member to add a concentrated moment'],
    ['dist', 'Linear q', 'Q', I('<path d="M3 5h14M4 5v8M10 5v8M16 5v8M2.5 11L4 13l1.5-2M8.5 11l1.5 2 1.5-2M14.5 11l1.5 2 1.5-2M3 15.5h14"/>'), 'Uniform or linearly varying (trapezoid / triangle) load — click a member'],
    ['para', 'Parabolic q', '', I('<path d="M3 13Q10 -3 17 13M5 9.5v3.5M10 5v8M15 9.5v3.5M3 15.5h14"/>'), 'Parabolic distributed load through three values (start, middle, end) — click a member'],
    ['func', 'Function q(x)', '', I('<path d="M3 13c2-9 4-9 5.5-4s3 5 4.5 0 2.5-5 4-2M3 15.5h14"/>'), 'Distributed load given by any formula q(x) — click a member'],
    ['', 'Results'], ['probe', 'Probe', 'P', I('<circle cx="10" cy="10" r="6"/><path d="M10 1v5M10 14v5M1 10h5M14 10h5"/>'), 'Click a point on a member or a joint to pin values there']
  ];
  function setTool(t) {
    tool = t; pending = null;
    document.querySelectorAll('#tools button').forEach(b => b.classList.toggle('on', b.dataset.tool === t));
    svg.classList.toggle('sel', t === 'select'); svg.classList.toggle('grab', t === 'pan');
    $('#hint').textContent = (TOOLS.find(x => x[0] === t) || [])[4] || ''; drawOverlay();
  }
  TOOLS.forEach(t => {
    if (!t[0]) { $('#tools').append(h('div', { class: 'sep' }, t[1])); return; }
    $('#tools').append(h('button', { 'data-tool': t[0], title: t[4], html: t[3] + `<span>${t[1]}</span>` + (t[2] ? `<kbd>${t[2]}</kbd>` : ''), onclick: () => setTool(t[0]) }));
  });

  // ---------- pointer interaction
  function pos(e) { const r = svg.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
  svg.addEventListener('contextmenu', e => e.preventDefault());
  svg.addEventListener('wheel', e => {
    e.preventDefault(); const p = pos(e), w = toWorld(p[0], p[1]), f = Math.exp(-e.deltaY * 0.0015);
    view.z = Math.max(4, Math.min(2000, view.z * f)); view.ox = p[0] - w[0] * view.z; view.oy = p[1] + w[1] * view.z; render();
  }, { passive: false });
  svg.addEventListener('pointerdown', e => {
    const p = pos(e); mouse.x = p[0]; mouse.y = p[1];
    if (e.button === 2 && tool === 'member' && pending) { pending = null; drawOverlay(); return; }
    if (e.button === 1 || e.button === 2 || spaceDown || tool === 'pan') { drag = { kind: 'pan', x: p[0], y: p[1], ox: view.ox, oy: view.oy }; svg.setPointerCapture(e.pointerId); svg.classList.add('pan'); return; }
    if (e.button !== 0) return;
    const pn = pickNode(p[0], p[1]), pm = pn ? null : pickMember(p[0], p[1]), w = toWorld(p[0], p[1]);
    const station = () => { const t = snapT(pm); return [pm.g.a.x + pm.g.c * pm.g.L * t, pm.g.a.y + pm.g.s * pm.g.L * t, t]; };
    const here = () => { // node under the cursor, creating one (and splitting a member) if needed
      if (pn) return pn; const gx = snap(w[0]), gy = snap(w[1]);
      if (pm) { const g = pm.g, px = gx - g.a.x, py = gy - g.a.y, off = Math.abs(-px * g.s + py * g.c), al = px * g.c + py * g.s; if (off < 1e-6 && al > 1e-6 && al < g.L - 1e-6) return nodeAt(gx, gy); const q = station(); return nodeAt(q[0], q[1]); }
      return nodeAt(gx, gy);
    };
    switch (tool) {
      case 'select':
        if (pn) { sel = { kind: 'node', id: pn.id }; drag = { kind: 'node', id: pn.id, moved: false }; svg.setPointerCapture(e.pointerId); }
        else if (pm) sel = { kind: 'member', id: pm.m.id };
        else { sel = null; drag = { kind: 'pan', x: p[0], y: p[1], ox: view.ox, oy: view.oy }; svg.setPointerCapture(e.pointerId); }
        if (tab !== 'inspect') { tab = 'inspect'; setTab(); }
        render(); renderPanel(); break;
      case 'erase': if (pn) change(() => deleteNode(pn)); else if (pm) change(() => deleteMember(pm.m)); break;
      case 'node': change(() => { const n = here(); sel = { kind: 'node', id: n.id }; }); break;
      case 'member':
        change(() => {
          const n = here();
          if (pending && pending !== n.id && !model.members.some(m => (m.n1 === pending && m.n2 === n.id) || (m.n2 === pending && m.n1 === n.id))) { const m = addMember(pending, n.id); sel = { kind: 'member', id: m.id }; }
          pending = n.id;
        }); break;
      case 'pin': case 'roller': case 'fixed': case 'guide':
        if (pn || pm) change(() => { const n = here(); n.sup = n.sup === tool ? 'none' : tool; sel = { kind: 'node', id: n.id }; }); break;
      case 'spy': case 'spx': case 'spr':
        if (pn || pm) {
          const key = ({ spy: 'ky', spx: 'kx', spr: 'kr' })[tool], ci = ({ spy: 1, spx: 0, spr: 2 })[tool]; let msg = '';
          change(() => {
            const n = here(), sp = Solver.SUP[n.sup];
            if (sp && sp[ci]) msg = 'The support at ' + nName(n) + ' already holds that direction rigidly — change the support type first, then add the spring.';
            else n[key] = n[key] > 0 ? 0 : 1000;
            sel = { kind: 'node', id: n.id };
          });
          tab = 'inspect'; setTab(); renderPanel(); if (msg) $('#hint').textContent = msg;
        } break;
      case 'hinge':
        if (pn) change(() => toggleHingeAtNode(pn));
        else if (pm) change(() => { const i = pm.t < 0.5 ? 2 : 5; pm.m.rel[i] = pm.m.rel[i] ? 0 : 1; }); break;
      case 'force': case 'moment': {
        const base = tool === 'force' ? { Px: 0, Py: 10, M: 0 } : { Px: 0, Py: 0, M: 10 };
        if (pn) change(() => { addLoad(Object.assign({ type: 'point', node: pn.id }, base)); sel = { kind: 'node', id: pn.id }; });
        else if (pm) change(() => { addLoad(Object.assign({ type: 'point', member: pm.m.id, t: snapT(pm) }, base)); sel = { kind: 'member', id: pm.m.id }; });
        if (pn || pm) { tab = 'inspect'; setTab(); renderPanel(); } break;
      }
      case 'dist': case 'para': case 'func':
        if (pm) { const shp = ({ dist: 'linear', para: 'parabola', func: 'func' })[tool]; change(() => { newDist(shp, pm.m.id); sel = { kind: 'member', id: pm.m.id }; }); tab = 'inspect'; setTab(); renderPanel(); } break;
      case 'probe':
        if (pn) probe = { node: pn.id }; else if (pm) probe = { member: pm.m.id, t: snapT(pm) }; else break;
        mmQ = null; ilQ = null; if (tab === 'help') { tab = 'inspect'; setTab(); } refresh(); break;
    }
  });
  svg.addEventListener('pointermove', e => {
    const p = pos(e); mouse.x = p[0]; mouse.y = p[1];
    const w = toWorld(p[0], p[1]); $('#coords').textContent = `x ${fmt(snap(w[0]), 4)}   y ${fmt(snap(w[1]), 4)}`;
    if (drag && drag.kind === 'pan') { view.ox = drag.ox + p[0] - drag.x; view.oy = drag.oy + p[1] - drag.y; render(); return; }
    if (drag && drag.kind === 'node') {
      const n = node(drag.id), x = snap(w[0]), y = snap(w[1]);
      if (n && (n.x !== x || n.y !== y) && !model.nodes.some(q => q !== n && q.x === x && q.y === y)) { if (!drag.moved) { undo.push(JSON.stringify(model)); redo = []; drag.moved = true; } n.x = x; n.y = y; res = Solver.analyze(model); scales(); mm = null; il = null; render(); }
      return;
    }
    drawOverlay();
    const pm = pickMember(p[0], p[1], 10), rd = $('#read');
    if (pm && res && res.ok) {
      const a = res.at(pm.m.id, pm.t * pm.g.L, -1);
      rd.textContent = `Member ${mName(pm.m)} · x = ${fmt(a.x, 4)} · N = ${fF(a.N, 4)} · S = ${fF(a.S, 4)} · M = ${fF(a.M, 4)} · u = ${fD(a.ux, 4)} · v = ${fD(a.uy, 4)}`;
    } else rd.textContent = '';
  });
  function endDrag() { if (!drag) return; const d = drag; drag = null; svg.classList.remove('pan'); if (d.kind === 'node' && d.moved) refresh(); }
  svg.addEventListener('pointerup', endDrag); svg.addEventListener('pointercancel', endDrag);
  svg.addEventListener('dblclick', () => { if (tool === 'member') { pending = null; drawOverlay(); } });

  window.addEventListener('keydown', e => {
    if (/INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
    if (e.key === ' ') { spaceDown = true; e.preventDefault(); return; }
    if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') { e.preventDefault(); doUndo(); return; }
    if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'y') { e.preventDefault(); doRedo(); return; }
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.key === 'Escape') { if (pending) { pending = null; drawOverlay(); } else setTool('select'); return; }
    if ((e.key === 'Delete' || e.key === 'Backspace') && sel) { const s = sel; change(() => { if (s.kind === 'node') deleteNode(node(s.id)); else deleteMember(member(s.id)); }); return; }
    const arrows = { ArrowLeft: [60, 0], ArrowRight: [-60, 0], ArrowUp: [0, 60], ArrowDown: [0, -60] }[e.key];
    if (arrows) { e.preventDefault(); view.ox += arrows[0]; view.oy += arrows[1]; render(); return; }
    if (e.key === '+' || e.key === '=' || e.key === '-') { const f = e.key === '-' ? 1 / 1.2 : 1.2, cx = svg.clientWidth / 2, cy = svg.clientHeight / 2, w = toWorld(cx, cy); view.z = Math.max(4, Math.min(2000, view.z * f)); view.ox = cx - w[0] * view.z; view.oy = cy + w[1] * view.z; render(); return; }
    const t = TOOLS.find(x => x[2] && x[2].length === 1 && x[2].toLowerCase() === e.key.toLowerCase()); if (t) setTool(t[0]);
  });
  window.addEventListener('keyup', e => { if (e.key === ' ') spaceDown = false; });
  window.addEventListener('resize', () => render());

  function doUndo() { if (!undo.length) return; redo.push(JSON.stringify(model)); restore(undo.pop()); }
  function doRedo() { if (!redo.length) return; undo.push(JSON.stringify(model)); restore(redo.pop()); }
  function fit() {
    if (!model.nodes.length) { view.z = 60; view.ox = 140; view.oy = (svg.clientHeight || 600) - 160; return; }
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    model.nodes.forEach(n => { x0 = Math.min(x0, n.x); x1 = Math.max(x1, n.x); y0 = Math.min(y0, n.y); y1 = Math.max(y1, n.y); });
    const w = svg.clientWidth || 800, hh = svg.clientHeight || 600, mg = 110;
    view.z = Math.max(4, Math.min(160, (w - 2 * mg) / Math.max(x1 - x0, 1e-6), (hh - 2 * mg) / Math.max(y1 - y0, 1e-6)));
    view.ox = w / 2 - (x0 + x1) / 2 * view.z; view.oy = hh / 2 + (y0 + y1) / 2 * view.z;
  }

  // ---------- examples (several reproduce the lecture notes)
  const hingeBar = { rel: [0, 0, 1, 0, 0, 1] };
  const EX = {
    'Simply supported beam, uniform load': () => { const a = addNode(0, 0), b = addNode(6, 0); a.sup = 'pin'; b.sup = 'roller'; const m = addMember(a.id, b.id); addLoad({ type: 'dist', member: m.id, t1: 0, t2: 1, q1: 10, q2: 10, dir: 'Y' }); },
    'Gerber beam (Lecture 1)': () => {
      const xs = [0, 6, 7, 12, 13, 17], n = xs.map(x => addNode(x, 0)); n[0].sup = 'pin'; n[1].sup = 'roller'; n[4].sup = 'roller'; n[5].sup = 'roller';
      const m = []; for (let i = 0; i < 5; i++) m.push(addMember(n[i].id, n[i + 1].id)); m[1].rel[5] = 1; m[2].rel[5] = 1;
      addLoad({ type: 'point', member: m[0].id, t: 0.5, Px: 0, Py: 6, M: 0 }); addLoad({ type: 'point', member: m[2].id, t: 0.4, Px: 0, Py: 5, M: 0 }); addLoad({ type: 'point', member: m[4].id, t: 0.5, Px: 0, Py: 4, M: 0 });
    },
    'Portal frame (Lecture 3: Δ_B = 189/8·qa⁴/EI₀)': () => {
      const s = U(); s.E = s.A = s.I = 1; s.axialRigid = true;
      const A = addNode(0, 0), C = addNode(0, 3), D = addNode(2, 3), B = addNode(2, 0); A.sup = 'pin'; B.sup = 'roller';
      const m1 = addMember(A.id, C.id); addMember(C.id, D.id, { I: 2 }); addMember(D.id, B.id); addLoad({ type: 'dist', member: m1.id, t1: 0, t2: 1, q1: 1, q2: 1, dir: 'X' });
      probe = { node: B.id }; mmQ = { target: { node: B.id }, dir: 'x' };
    },
    'Three-hinged frame': () => {
      const A = addNode(0, 0), B = addNode(0, 4), C = addNode(3, 5), D = addNode(6, 4), E = addNode(6, 0); A.sup = 'pin'; E.sup = 'pin';
      addMember(A.id, B.id); const m2 = addMember(B.id, C.id); m2.rel[5] = 1; const m3 = addMember(C.id, D.id); addMember(D.id, E.id);
      addLoad({ type: 'dist', member: m2.id, t1: 0, t2: 1, q1: 12, q2: 12, dir: 'proj' }); addLoad({ type: 'dist', member: m3.id, t1: 0, t2: 1, q1: 12, q2: 12, dir: 'proj' }); addLoad({ type: 'point', node: B.id, Px: 20, Py: 0, M: 0 });
    },
    'Truss, 4 panels': () => {
      const b = [0, 3, 6, 9, 12].map(x => addNode(x, 0)), t = [3, 6, 9].map(x => addNode(x, 3)); b[0].sup = 'pin'; b[4].sup = 'roller';
      const bar = (p, q) => addMember(p.id, q.id, { rel: hingeBar.rel.slice() });
      for (let i = 0; i < 4; i++) bar(b[i], b[i + 1]); bar(t[0], t[1]); bar(t[1], t[2]); for (let i = 0; i < 3; i++) bar(b[i + 1], t[i]);
      bar(b[0], t[0]); bar(t[0], b[2]); bar(b[2], t[2]); bar(t[2], b[4]);
      [1, 2, 3].forEach(i => addLoad({ type: 'point', node: b[i].id, Px: 0, Py: 20, M: 0 }));
    },
    'Continuous beam (indeterminate)': () => {
      const n = [0, 5, 11, 15].map(x => addNode(x, 0)); n[0].sup = 'pin'; n[1].sup = n[2].sup = n[3].sup = 'roller';
      const m = [0, 1, 2].map(i => addMember(n[i].id, n[i + 1].id)); m.forEach(q => addLoad({ type: 'dist', member: q.id, t1: 0, t2: 1, q1: 15, q2: 15, dir: 'Y' })); addLoad({ type: 'point', member: m[1].id, t: 0.5, Px: 0, Py: 40, M: 0 });
    },
    'Portal frame with sway (slope-deflection, Cross)': () => {
      const s = U(); s.axialRigid = true;
      const A = addNode(0, 0), B = addNode(0, 4), C = addNode(6, 4), D = addNode(6, 0); A.sup = 'fixed'; D.sup = 'pin';
      addMember(A.id, B.id); const m2 = addMember(B.id, C.id, { I: 2 * s.I }); addMember(C.id, D.id);
      addLoad({ type: 'dist', member: m2.id, t1: 0, t2: 1, q1: 12, q2: 12, dir: 'Y' }); addLoad({ type: 'point', node: B.id, Px: 20, Py: 0, M: 0 });
    },
    'Beam on springs (vertical, horizontal, rotational)': () => {
      const s = U(); s.E = s.A = s.I = 1;
      const a = addNode(0, 0), b = addNode(4, 0), c = addNode(8, 0); a.sup = 'pin'; a.kr = 2; b.ky = 0.5; c.ky = 1; c.kx = 0.25;
      const m1 = addMember(a.id, b.id), m2 = addMember(b.id, c.id);
      addLoad({ type: 'dist', member: m1.id, t1: 0, t2: 1, q1: 1, q2: 1, dir: 'Y' }); addLoad({ type: 'dist', member: m2.id, t1: 0, t2: 1, q1: 1, q2: 1, dir: 'Y' });
      addLoad({ type: 'point', node: c.id, Px: 1, Py: 0, M: 0 }); show.def = true;
    },
    'Parabolic and function loads': () => {
      const n = [0, 6, 12].map(x => addNode(x, 0)); n[0].sup = 'pin'; n[1].sup = 'roller'; n[2].sup = 'roller';
      const m1 = addMember(n[0].id, n[1].id), m2 = addMember(n[1].id, n[2].id);
      addLoad({ type: 'dist', member: m1.id, t1: 0, t2: 1, q1: 0, qm: 12, q2: 0, shape: 'parabola', dir: 'Y' });
      addLoad({ type: 'dist', member: m2.id, t1: 0, t2: 1, q1: 0, q2: 0, shape: 'func', expr: '2 + 8*(x/L)^3', dir: 'Y' });
    },
    'Temperature and settlement (frame)': () => {
      const A = addNode(0, 0), B = addNode(0, 4), C = addNode(5, 4), D = addNode(5, 0); A.sup = 'pin'; D.sup = 'roller'; D.dy = -0.02;
      addMember(A.id, B.id); addMember(B.id, C.id, { Tt: 30, Tb: 10, h: 0.4 }); addMember(C.id, D.id); show.def = true;
    }
  };
  Object.keys(EX).forEach(k => $('#examples').append(h('option', { value: k }, k)));
  function loadExample(k) {
    undo.push(JSON.stringify(model)); redo = [];
    model = blank(); sel = null; probe = null; mmQ = null; ilQ = null; pending = null; EX[k](); fit(); refresh();
  }
  $('#examples').addEventListener('change', e => { if (e.target.value) loadExample(e.target.value); e.target.value = ''; });
  $('#bNew').addEventListener('click', () => { change(() => { model = blank(); sel = null; probe = null; mmQ = null; ilQ = null; }); fit(); render(); });
  $('#bUndo').addEventListener('click', doUndo); $('#bRedo').addEventListener('click', doRedo);
  $('#bFit').addEventListener('click', () => { fit(); render(); });
  $('#bPrint').addEventListener('click', () => { tab = 'calc'; setTab(); renderPanel(); window.print(); });
  function download(name, textData, type) { const a = h('a', { href: URL.createObjectURL(new Blob([textData], { type })), download: name }); document.body.append(a); a.click(); a.remove(); }
  $('#bSave').addEventListener('click', () => download('structure.json', JSON.stringify(model, null, 1), 'application/json'));
  $('#bSvg').addEventListener('click', () => {
    const w = svg.clientWidth, hh = svg.clientHeight;
    download('structure.svg', `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${hh}" viewBox="0 0 ${w} ${hh}"><style>${$('#svgcss').textContent.replace(':root[data-theme="dark"]', theme === 'dark' ? 'svg' : 'svg.unused').replace(':root', 'svg')}</style>${svg.innerHTML}</svg>`, 'image/svg+xml');
  });
  $('#bOpen').addEventListener('click', () => $('#file').click());
  $('#file').addEventListener('change', e => {
    const f = e.target.files[0]; if (!f) return;
    f.text().then(t => {
      try { const m = JSON.parse(t); if (!m || !Array.isArray(m.nodes) || !Array.isArray(m.members) || !Array.isArray(m.loads)) throw 0; m.settings = Object.assign(blank().settings, m.settings || {}); undo.push(JSON.stringify(model)); model = m; sel = null; fit(); refresh(); }
      catch (err) { alert('That file is not a saved structure.'); }
    }); e.target.value = '';
  });

  // ---------- theme
  let theme = null;
  try { theme = localStorage.getItem(LS + '-theme'); } catch (e) { /* ignore */ }
  if (theme !== 'dark' && theme !== 'light') theme = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
  function applyTheme() { document.documentElement.dataset.theme = theme; $('#bTheme').textContent = theme === 'dark' ? 'Light theme' : 'Dark theme'; }
  $('#bTheme').addEventListener('click', () => { theme = theme === 'dark' ? 'light' : 'dark'; try { localStorage.setItem(LS + '-theme', theme); } catch (e) { /* ignore */ } applyTheme(); });
  applyTheme();

  // ---------- start
  model = blank();
  let restored = false;
  try { const t = localStorage.getItem(LS); if (t) { const m = JSON.parse(t); if (m && m.members && m.members.length) { m.settings = Object.assign(blank().settings, m.settings || {}); model = m; restored = true; } } } catch (e) { /* ignore */ }
  setTool('select');
  requestAnimationFrame(() => { if (!restored) { EX['Simply supported beam, uniform load'](); } fit(); refresh(); });
})();
