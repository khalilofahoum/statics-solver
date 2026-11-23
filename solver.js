/* Plane frame / beam / truss solver.
 * Theory: direct stiffness method (Kirsch ch. 4.3), internal forces by statics along each member,
 * displacements also by the Maxwell–Mohr unit-load integral (Kirsch 2.2 / Lectures 2–4),
 * influence lines by a moving unit load (Kirsch 2.1, 3.4).
 * Sign convention (course): N > 0 tension; S = dM/dx, q = -dS/dx; M > 0 tension at the bottom
 * (dashed) fibre. Local x runs from start node to end node, local y is 90° counter-clockwise from x,
 * the "bottom" fibre is the -y side.
 */
(function (root) {
  'use strict';

  var SUP = { pin: [1, 1, 0], roller: [0, 1, 0], fixed: [1, 1, 1], guide: [0, 1, 1] };

  function zeros(n, m) { var a = []; for (var i = 0; i < n; i++) a.push(new Float64Array(m)); return a; }
  function pval(p, x) { var r = 0; for (var i = p.length - 1; i >= 0; i--) r = r * x + p[i]; return r; }
  function pint(p, c0) { var r = [c0 || 0]; for (var i = 0; i < p.length; i++) r.push(p[i] / (i + 1)); return r; }
  function pscale(p, k) { return p.map(function (v) { return v * k; }); }
  function padd(a, b) { var r = []; for (var i = 0; i < Math.max(a.length, b.length); i++) r.push((a[i] || 0) + (b[i] || 0)); return r; }
  // q(x) = p(x - a)
  function pshift(p, a) {
    var n = p.length, q = new Array(n).fill(0);
    for (var k = 0; k < n; k++) {
      var binom = 1;
      for (var j = 0; j <= k; j++) { q[j] += p[k] * binom * Math.pow(-a, k - j); binom = binom * (k - j) / (j + 1); }
    }
    return q;
  }

  function invert(A) {
    var n = A.length, i, j, k, M = [], mx = 0;
    for (i = 0; i < n; i++) { M.push([]); for (j = 0; j < n; j++) { M[i].push(A[i][j]); mx = Math.max(mx, Math.abs(A[i][j])); } for (j = 0; j < n; j++) M[i].push(i === j ? 1 : 0); }
    for (k = 0; k < n; k++) {
      var p = k; for (i = k + 1; i < n; i++) if (Math.abs(M[i][k]) > Math.abs(M[p][k])) p = i;
      if (Math.abs(M[p][k]) <= 1e-12 * mx) return null;
      var t = M[k]; M[k] = M[p]; M[p] = t;
      var d = M[k][k]; for (j = 0; j < 2 * n; j++) M[k][j] /= d;
      for (i = 0; i < n; i++) if (i !== k) { var f = M[i][k]; if (f) for (j = 0; j < 2 * n; j++) M[i][j] -= f * M[k][j]; }
    }
    return M.map(function (r) { return r.slice(n); });
  }

  function solveLinear(A, b, scale) {
    var n = b.length, i, j, k;
    for (k = 0; k < n; k++) {
      var p = k, mx = Math.abs(A[k][k]);
      for (i = k + 1; i < n; i++) if (Math.abs(A[i][k]) > mx) { mx = Math.abs(A[i][k]); p = i; }
      if (mx <= 1e-10 * scale) return null;
      if (p !== k) { var t = A[k]; A[k] = A[p]; A[p] = t; var tb = b[k]; b[k] = b[p]; b[p] = tb; }
      for (i = k + 1; i < n; i++) {
        var f = A[i][k] / A[k][k];
        if (f) { for (j = k; j < n; j++) A[i][j] -= f * A[k][j]; b[i] -= f * b[k]; }
      }
    }
    var x = new Float64Array(n);
    for (i = n - 1; i >= 0; i--) { var s = b[i]; for (j = i + 1; j < n; j++) s -= A[i][j] * x[j]; x[i] = s / A[i][i]; }
    return x;
  }

  function kLocal(EA, EI, L) {
    var k = zeros(6, 6), a = EA / L, b = 12 * EI / (L * L * L), c = 6 * EI / (L * L), d = 4 * EI / L, e = 2 * EI / L;
    k[0][0] = a; k[0][3] = -a; k[3][0] = -a; k[3][3] = a;
    k[1][1] = b; k[1][2] = c; k[1][4] = -b; k[1][5] = c;
    k[2][1] = c; k[2][2] = d; k[2][4] = -c; k[2][5] = e;
    k[4][1] = -b; k[4][2] = -c; k[4][4] = b; k[4][5] = -c;
    k[5][1] = c; k[5][2] = e; k[5][4] = -c; k[5][5] = d;
    return k;
  }

  // Static condensation of released end DOFs (hinges, sliders).
  function condense(k, f0, rel) {
    var c = [], r = [], i;
    for (i = 0; i < 6; i++) (rel[i] ? c : r).push(i);
    if (!c.length) return { k: k, f0: f0, rec: null };
    var inv = invert(c.map(function (a) { return c.map(function (b) { return k[a][b]; }); }));
    if (!inv) return null;
    var kk = zeros(6, 6), ff = new Float64Array(6), nc = c.length;
    r.forEach(function (a) {
      var row = new Float64Array(nc), p, q;
      for (q = 0; q < nc; q++) for (p = 0; p < nc; p++) row[q] += k[a][c[p]] * inv[p][q];
      r.forEach(function (b) { var s = k[a][b]; for (q = 0; q < nc; q++) s -= row[q] * k[c[q]][b]; kk[a][b] = s; });
      var s2 = f0[a]; for (q = 0; q < nc; q++) s2 -= row[q] * f0[c[q]]; ff[a] = s2;
    });
    return {
      k: kk, f0: ff,
      rec: function (d) {
        var rhs = c.map(function (cq) { var s = f0[cq]; r.forEach(function (b) { s += k[cq][b] * d[b]; }); return s; });
        for (var p = 0; p < nc; p++) { var v = 0; for (var q = 0; q < nc; q++) v -= inv[p][q] * rhs[q]; d[c[p]] = v; }
      }
    };
  }

  function loadToLocal(q, dir, c, s) {
    switch (dir) {
      case 'perp': return [0, -q];
      case 'axial': return [q, 0];
      case 'X': return [q * c, -q * s];
      case 'proj': var g = -q * Math.abs(c); return [g * s, g * c];
      default: return [-q * s, -q * c];
    }
  }

  // Safe arithmetic parser for user load functions q(x): + - * / ^, parentheses, x, L, pi, e and common functions.
  var FN = { sin: Math.sin, cos: Math.cos, tan: Math.tan, exp: Math.exp, ln: Math.log, log: Math.log10, sqrt: Math.sqrt, abs: Math.abs };
  function compileExpr(src) {
    var s = String(src == null ? '' : src).replace(/\s+/g, '').replace(/\*\*/g, '^').toLowerCase(), i = 0;
    if (!s) throw new Error('empty expression');
    function expr() { var a = term(); while (s[i] === '+' || s[i] === '-') { a = bin(s[i++], a, term()); } return a; }
    function bin(op, a, b) {
      return op === '+' ? function (v) { return a(v) + b(v); } : op === '-' ? function (v) { return a(v) - b(v); } :
        op === '*' ? function (v) { return a(v) * b(v); } : function (v) { return a(v) / b(v); };
    }
    function term() {
      var a = unary();
      for (;;) {
        if (s[i] === '*' || s[i] === '/') a = bin(s[i++], a, unary());
        else if (i < s.length && /[a-z0-9.(]/.test(s[i])) a = bin('*', a, pow()); // implicit product: 2x, 3(x+1)
        else break;
      }
      return a;
    }
    function unary() { if (s[i] === '-') { i++; var a = unary(); return function (v) { return -a(v); }; } if (s[i] === '+') { i++; return unary(); } return pow(); }
    function pow() { var a = atom(); if (s[i] === '^') { i++; var b = unary(); return function (v) { return Math.pow(a(v), b(v)); }; } return a; }
    function atom() {
      if (s[i] === '(') { i++; var a = expr(); if (s[i] !== ')') throw new Error('missing )'); i++; return a; }
      var m = /^(\d+\.?\d*|\.\d+)(e[+-]?\d+)?/.exec(s.slice(i));
      if (m) { i += m[0].length; var num = parseFloat(m[0]); return function () { return num; }; }
      m = /^[a-z]+/.exec(s.slice(i));
      if (!m) throw new Error('unexpected "' + (s[i] || 'end') + '"');
      var id = m[0];
      if (FN[id] && s[i + id.length] === '(') { i += id.length + 1; var arg = expr(), fn = FN[id]; if (s[i] !== ')') throw new Error('missing )'); i++; return function (v) { return fn(arg(v)); }; }
      if (id.slice(0, 2) === 'pi') { i += 2; return function () { return Math.PI; }; }
      i += 1;
      if (id[0] === 'x') return function (v) { return v.x; };
      if (id[0] === 'l') return function (v) { return v.L; };
      if (id[0] === 'e') return function () { return Math.E; };
      throw new Error('unknown name "' + id + '"');
    }
    var E = expr(); if (i < s.length) throw new Error('unexpected "' + s[i] + '"');
    return function (x, L) { var r = E({ x: x, L: L }); return isFinite(r) ? r : 0; };
  }
  // Intensity of a distributed load along its range; tau = 0 at the lower end of the range, 1 at the upper end.
  // linear: q1..q2 · parabola: through q1, qm (middle), q2 · func: q(x), x from the start of the load, L its length
  function distShape(l, span) {
    var fw = l.t1 <= l.t2, q1 = +(fw ? l.q1 : l.q2) || 0, q2 = +(fw ? l.q2 : l.q1) || 0;
    if (l.shape === 'parabola') { var qm = +l.qm || 0; return { poly: [q1, -3 * q1 + 4 * qm - q2, 2 * q1 - 4 * qm + 2 * q2] }; }
    if (l.shape === 'func') {
      var f; try { f = compileExpr(l.expr); } catch (err) { return { poly: [0], err: err.message }; }
      var Lx = +l.Lx > 0 ? +l.Lx : span, xo = +l.xo || 0, rev = !!l.rev;
      return { fn: function (tau) { return f(xo + (rev ? 1 - tau : tau) * span, Lx); } };
    }
    return { poly: [q1, q2 - q1] };
  }
  function distQ(l, tau, span) { var sh = distShape(l, span); return sh.fn ? sh.fn(tau) : pval(sh.poly, tau); }

  // Degree of static indeterminacy: n = 3m + r - 3j - c
  function determinacy(model) {
    var m = model.members.length, used = {}, ends = {};
    model.members.forEach(function (mb) {
      [[mb.n1, 0], [mb.n2, 3]].forEach(function (e) {
        used[e[0]] = 1; var o = ends[e[0]] || (ends[e[0]] = { k: 0, h: 0, other: 0 });
        o.k++; var rel = mb.rel || []; if (rel[e[1] + 2]) o.h++; if (rel[e[1]]) o.other++; if (rel[e[1] + 1]) o.other++;
      });
    });
    var j = 0, r = 0, c = 0;
    model.nodes.forEach(function (n) {
      if (!used[n.id]) return; j++;
      var s = n.fix || SUP[n.sup] || [0, 0, 0];
      r += s[0] + s[1] + s[2];
      if (n.kx > 0 && !s[0]) r++; if (n.ky > 0 && !s[1]) r++; if (n.kr > 0 && !s[2]) r++;
      var o = ends[n.id], rotHeld = s[2] || n.kr > 0;
      c += o.other + ((o.h === o.k && !rotHeld) ? Math.max(o.k - 1, 0) : o.h);
    });
    return { m: m, j: j, r: r, c: c, n: 3 * m + r - 3 * j - c };
  }

  function analyze(model, opt) {
    opt = opt || {};
    var res = { ok: false, errors: [], warnings: [] };
    var nodes = model.nodes, members = model.members, st = model.settings || {};
    var loads = opt.loads || model.loads || [], virt = !!opt.virtual;
    if (!members.length) { res.errors.push('Draw at least one member to analyse.'); return res; }
    res.det = determinacy(model);

    var nById = new Map(nodes.map(function (n) { return [n.id, n]; }));
    var mn = [], nIdx = new Map(), usedNode = {};
    members.forEach(function (m) { usedNode[m.n1] = 1; usedNode[m.n2] = 1; });
    nodes.forEach(function (n) {
      var sup = SUP[n.sup] ? n.sup : null;
      nIdx.set(n.id, mn.length);
      mn.push({ x: n.x, y: n.y, beta: sup ? (n.ang || 0) * Math.PI / 180 : 0, node: n, orphan: !usedNode[n.id] });
    });

    var G = new Map(), Lsum = 0, EImax = 0, bad = false;
    members.forEach(function (m, i) {
      var a = nById.get(m.n1), b = nById.get(m.n2);
      if (!a || !b) { res.errors.push('Member ' + (i + 1) + ' is missing a node.'); bad = true; return; }
      var dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy);
      if (L < 1e-9) { res.errors.push('Member ' + (i + 1) + ' has zero length.'); bad = true; return; }
      if (!(m.E > 0 && m.A > 0 && m.I > 0)) { res.errors.push('Member ' + (i + 1) + ' needs positive E, A and I.'); bad = true; return; }
      G.set(m.id, { L: L, c: dx / L, s: dy / L }); Lsum += L; EImax = Math.max(EImax, m.E * m.I);
    });
    if (bad) return res;
    var EArigid = st.axialRigid ? 1e7 * EImax / Math.pow(Lsum / members.length, 2) : 0;

    // --- mesh: split members at point loads, distributed-load ends and requested stations
    var splits = new Map(members.map(function (m) { return [m.id, [0, 1]]; }));
    function addSplit(mid, t) { var a = splits.get(mid); if (a && t > 1e-9 && t < 1 - 1e-9) a.push(t); }
    (opt.splitLoads || []).concat(loads).forEach(function (l) {
      if (l.member == null) return;
      if (l.type === 'point') addSplit(l.member, l.t);
      else if (l.type === 'dist') {
        addSplit(l.member, l.t1); addSplit(l.member, l.t2);
        if (l.shape === 'func') for (var k = 1; k < 12; k++) addSplit(l.member, l.t1 + (l.t2 - l.t1) * k / 12);
      }
    });
    (opt.extraSplits || []).forEach(function (s) { addSplit(s.member, s.t); });

    var els = [], memEls = new Map(), stationNode = new Map();
    members.forEach(function (m) {
      var g = G.get(m.id), a = nById.get(m.n1), raw = splits.get(m.id).slice().sort(function (p, q) { return p - q; }), ts = [];
      raw.forEach(function (t) { if (!ts.length || t - ts[ts.length - 1] > 1e-9) ts.push(t); });
      var idx = [nIdx.get(m.n1)], i;
      for (i = 1; i < ts.length - 1; i++) {
        stationNode.set(m.id + ':' + ts[i].toFixed(9), mn.length); idx.push(mn.length);
        mn.push({ x: a.x + g.c * g.L * ts[i], y: a.y + g.s * g.L * ts[i], beta: 0, node: null, mid: m.id, t: ts[i] });
      }
      idx.push(nIdx.get(m.n2));
      var list = [], rel = m.rel || [], alpha = m.alpha || 0;
      for (i = 0; i < ts.length - 1; i++) {
        var r = [0, 0, 0, 0, 0, 0];
        if (i === 0) { r[0] = rel[0] ? 1 : 0; r[1] = rel[1] ? 1 : 0; r[2] = rel[2] ? 1 : 0; }
        if (i === ts.length - 2) { r[3] = rel[3] ? 1 : 0; r[4] = rel[4] ? 1 : 0; r[5] = rel[5] ? 1 : 0; }
        var e = {
          m: m, first: i === 0, last: i === ts.length - 2, i: idx[i], j: idx[i + 1], ta: ts[i], tb: ts[i + 1], xa: ts[i] * g.L, L: (ts[i + 1] - ts[i]) * g.L, c: g.c, s: g.s,
          EA: EArigid || m.E * m.A, EI: m.E * m.I, rel: r, wx: [0], wy: [0], eps0: 0, kap0: 0
        };
        if (!virt) {
          e.eps0 = alpha * ((+m.Tt || 0) + (+m.Tb || 0)) / 2 + (+m.dL || 0) / g.L;
          if (m.h > 0) e.kap0 = alpha * ((+m.Tb || 0) - (+m.Tt || 0)) / m.h;
          else if (alpha && (+m.Tt || 0) !== (+m.Tb || 0) && i === 0) res.warnings.push('Member ' + (members.indexOf(m) + 1) + ': a section depth h is needed for the temperature gradient.');
        }
        els.push(e); list.push(e);
      }
      memEls.set(m.id, list);
    });

    var ndof = mn.length * 3, K = zeros(ndof, ndof), F = new Float64Array(ndof);
    var applied = { Fx: 0, Fy: 0, Mo: 0 };
    function nodalLoad(ni, Fx, Fy, M) {
      var n = mn[ni], cb = Math.cos(n.beta), sb = Math.sin(n.beta);
      F[3 * ni] += cb * Fx + sb * Fy; F[3 * ni + 1] += -sb * Fx + cb * Fy; F[3 * ni + 2] += M;
      applied.Fx += Fx; applied.Fy += Fy; applied.Mo += M + n.x * Fy - n.y * Fx;
    }
    loads.forEach(function (l) {
      if (l.type === 'point') {
        var ni;
        if (l.node != null) ni = nIdx.get(l.node);
        else {
          var m = members.find(function (mm) { return mm.id === l.member; }); if (!m) return;
          ni = l.t <= 1e-9 ? nIdx.get(m.n1) : l.t >= 1 - 1e-9 ? nIdx.get(m.n2) : stationNode.get(l.member + ':' + l.t.toFixed(9));
        }
        if (ni == null) return;
        if (mn[ni].orphan) { res.warnings.push('A load on an unconnected node was ignored.'); return; }
        nodalLoad(ni, +l.Px || 0, -(+l.Py || 0), +l.M || 0);
      } else if (l.type === 'dist') {
        var list = memEls.get(l.member); if (!list) return;
        var lo = Math.min(l.t1, l.t2), hi = Math.max(l.t1, l.t2);
        if (hi - lo < 1e-9) return;
        var sh = distShape(l, (hi - lo) * G.get(l.member).L);
        if (sh.err) res.warnings.push('Load function "' + l.expr + '" could not be read: ' + sh.err);
        list.forEach(function (e) {
          if (e.ta < lo - 1e-9 || e.tb > hi + 1e-9) return;
          var ta = (e.ta - lo) / (hi - lo), dt = (e.tb - e.ta) / (hi - lo), cs;
          if (sh.poly) cs = pshift(sh.poly.map(function (c, k) { return c * Math.pow(dt / e.L, k); }), -ta * e.L / dt);
          else { // cubic through four points of q(x) on this short piece
            var f0 = sh.fn(ta), f1 = sh.fn(ta + dt / 3), f2 = sh.fn(ta + 2 * dt / 3), f3 = sh.fn(ta + dt);
            cs = [f0, (-11 * f0 + 18 * f1 - 9 * f2 + 2 * f3) / 2 / e.L, (18 * f0 - 45 * f1 + 36 * f2 - 9 * f3) / 2 / (e.L * e.L), (-9 * f0 + 27 * f1 - 27 * f2 + 9 * f3) / 2 / (e.L * e.L * e.L)];
          }
          var un = loadToLocal(1, l.dir, e.c, e.s);
          e.wx = padd(e.wx, pscale(cs, un[0])); e.wy = padd(e.wy, pscale(cs, un[1]));
        });
      }
    });

    // --- element matrices and assembly
    for (var ei = 0; ei < els.length; ei++) {
      var e = els[ei], L = e.L;
      e.k = kLocal(e.EA, e.EI, L);
      // equivalent joint loads of the member load (Hermite shape functions, Gauss integration) and its resultant
      var eq = [0, 0, 0, 0, 0, 0], na = mn[e.i], hL = L / 2;
      for (var gi = 0; gi < 5; gi++) {
        var sg = hL * (1 + GX[gi]), wt = GW[gi] * hL, xi = sg / L, pp = pval(e.wx, sg), ww = pval(e.wy, sg);
        eq[0] += wt * pp * (1 - xi); eq[3] += wt * pp * xi;
        eq[1] += wt * ww * (1 - 3 * xi * xi + 2 * xi * xi * xi); eq[2] += wt * ww * L * (xi - 2 * xi * xi + xi * xi * xi);
        eq[4] += wt * ww * (3 * xi * xi - 2 * xi * xi * xi); eq[5] += wt * ww * L * (-xi * xi + xi * xi * xi);
        var fx = (pp * e.c - ww * e.s) * wt, fy = (pp * e.s + ww * e.c) * wt, px = na.x + e.c * sg, py = na.y + e.s * sg;
        applied.Fx += fx; applied.Fy += fy; applied.Mo += px * fy - py * fx;
      }
      eq[0] -= e.EA * e.eps0; eq[3] += e.EA * e.eps0; eq[2] -= e.EI * e.kap0; eq[5] += e.EI * e.kap0;
      e.f0 = new Float64Array(eq.map(function (v) { return -v; }));
      // prescribed end forces on released DOFs (unit redundants of the force method)
      var f0c = e.f0, efs = (opt.endForces || []).filter(function (q) { return q.member === e.m.id && (q.dof < 3 ? e.first : e.last) && e.rel[q.dof]; });
      if (efs.length) { f0c = Float64Array.from(e.f0); efs.forEach(function (q) { f0c[q.dof] -= q.value; }); }
      e.efs = efs;
      e.cd = condense(e.k, f0c, e.rel);
      if (!e.cd) { res.errors.push('Member ' + (members.indexOf(e.m) + 1) + ' has too many end releases — it is a mechanism by itself.'); return res; }
      var T = zeros(6, 6);
      [[e.i, 0], [e.j, 3]].forEach(function (p) {
        var g = Math.atan2(e.s, e.c) - mn[p[0]].beta, cg = Math.cos(g), sg = Math.sin(g), o = p[1];
        T[o][o] = cg; T[o][o + 1] = sg; T[o + 1][o] = -sg; T[o + 1][o + 1] = cg; T[o + 2][o + 2] = 1;
      });
      e.T = T;
      e.efs.forEach(function (q) { for (var a2 = 0; a2 < 6; a2++) F[[3 * e.i, 3 * e.i + 1, 3 * e.i + 2, 3 * e.j, 3 * e.j + 1, 3 * e.j + 2][a2]] -= T[q.dof][a2] * q.value; });
      var dofs = [3 * e.i, 3 * e.i + 1, 3 * e.i + 2, 3 * e.j, 3 * e.j + 1, 3 * e.j + 2], kc = e.cd.k, a, b, p2, q2;
      var kT = zeros(6, 6);
      for (a = 0; a < 6; a++) for (b = 0; b < 6; b++) { var s = 0; for (p2 = 0; p2 < 6; p2++) s += kc[a][p2] * T[p2][b]; kT[a][b] = s; }
      for (a = 0; a < 6; a++) {
        for (b = 0; b < 6; b++) { var s3 = 0; for (q2 = 0; q2 < 6; q2++) s3 += T[q2][a] * kT[q2][b]; K[dofs[a]][dofs[b]] += s3; }
        var fs = 0; for (q2 = 0; q2 < 6; q2++) fs += T[q2][a] * e.cd.f0[q2]; F[dofs[a]] -= fs;
      }
    }

    // --- supports, springs, settlements
    var fixed = new Int8Array(ndof), presc = new Float64Array(ndof), springK = new Float64Array(ndof);
    mn.forEach(function (n, ni) {
      if (n.orphan) { fixed[3 * ni] = fixed[3 * ni + 1] = fixed[3 * ni + 2] = 2; return; }
      if (!n.node) return;
      var s = n.node.fix || SUP[n.node.sup] || [0, 0, 0], nd = n.node;
      var kk = [+nd.kx || 0, +nd.ky || 0, +nd.kr || 0], dd = [+nd.dx || 0, +nd.dy || 0, +nd.dr || 0];
      for (var c = 0; c < 3; c++) {
        if (s[c]) { fixed[3 * ni + c] = 1; if (!virt) presc[3 * ni + c] = dd[c]; }
        else if (kk[c] > 0) { springK[3 * ni + c] = kk[c]; K[3 * ni + c][3 * ni + c] += kk[c]; }
      }
    });
    var maxd = 0, i, j;
    for (i = 0; i < ndof; i++) maxd = Math.max(maxd, Math.abs(K[i][i]));
    for (i = 2; i < ndof; i += 3) if (!fixed[i] && Math.abs(K[i][i]) <= 1e-13 * maxd) {
      fixed[i] = 2; // rotation of a fully hinged joint: not a structural unknown
      if (Math.abs(F[i]) > 1e-12) res.warnings.push('A moment applied to a fully hinged joint was ignored — apply it to a member end instead.');
    }
    var free = []; for (i = 0; i < ndof; i++) if (!fixed[i]) free.push(i);
    var d = new Float64Array(ndof); for (i = 0; i < ndof; i++) if (fixed[i] === 1) d[i] = presc[i];
    if (free.length) {
      var A = zeros(free.length, free.length), bvec = new Float64Array(free.length);
      for (i = 0; i < free.length; i++) {
        var fi = free[i], rhs = F[fi];
        for (j = 0; j < ndof; j++) if (fixed[j] === 1 && presc[j]) rhs -= K[fi][j] * presc[j];
        bvec[i] = rhs;
        for (j = 0; j < free.length; j++) A[i][j] = K[fi][free[j]];
      }
      res.kin = free.length; res.kinJ = free.filter(function (q) { return mn[(q - q % 3) / 3].node; }).length;
      if (free.length <= 30) res.sys = {
        labels: free.map(function (q) { var nn = mn[(q - q % 3) / 3]; return nn.node ? { node: nn.node.id, comp: q % 3 } : { member: nn.mid, t: nn.t, comp: q % 3 }; }),
        K: A.map(function (r) { return Array.from(r); }), F: Array.from(bvec)
      };
      var sol = solveLinear(A, bvec, maxd);
      if (!sol) {
        res.unstable = true;
        res.errors.push(res.det.n < 0
          ? 'The structure is a mechanism: it has ' + (-res.det.n) + ' too few restraints (n = ' + res.det.n + ').'
          : 'The structure is geometrically unstable (a mechanism) even though the count gives n = ' + res.det.n + '. Check the supports and hinges.');
        return res;
      }
      for (i = 0; i < free.length; i++) d[free[i]] = sol[i];
      if (res.sys) res.sys.d = Array.from(sol);
    }

    // --- reactions
    res.reactions = []; res.springs = []; res.supDofs = [];
    var react = { Fx: 0, Fy: 0, Mo: 0 };
    mn.forEach(function (n, ni) {
      if (!n.node || n.orphan) return;
      var R = [0, 0, 0], any = false, sp = [0, 0, 0], anySp = false;
      for (var c = 0; c < 3; c++) {
        var q = 3 * ni + c;
        if (fixed[q] === 1) {
          var s = -F[q]; for (var jj = 0; jj < ndof; jj++) s += K[q][jj] * d[jj];
          R[c] = s; any = true; res.supDofs.push({ node: n.node.id, comp: c, R: s, presc: presc[q] });
        } else if (springK[q] > 0) {
          sp[c] = -springK[q] * d[q]; anySp = true;
          res.springs.push({ node: n.node.id, comp: c, k: springK[q], d: d[q], F: sp[c] });
        }
      }
      var cb = Math.cos(n.beta), sb = Math.sin(n.beta);
      [[R, any, false], [sp, anySp, true]].forEach(function (p) {
        if (!p[1]) return;
        var v = p[0], o = { node: n.node.id, x: n.x, y: n.y, Rx: cb * v[0] - sb * v[1], Ry: sb * v[0] + cb * v[1], M: v[2], spring: p[2], local: [v[0], v[1], v[2]], beta: n.beta };
        res.reactions.push(o);
        react.Fx += o.Rx; react.Fy += o.Ry; react.Mo += o.M + n.x * o.Ry - n.y * o.Rx;
      });
    });
    res.equilibrium = { applied: applied, react: react };

    // --- node displacements (global axes)
    res.disp = new Map();
    mn.forEach(function (n, ni) {
      if (!n.node) return;
      var cb = Math.cos(n.beta), sb = Math.sin(n.beta), a = d[3 * ni], b = d[3 * ni + 1];
      res.disp.set(n.node.id, { u: cb * a - sb * b, v: sb * a + cb * b, r: d[3 * ni + 2] });
    });

    // --- member internal forces and elastic line, segment by segment
    res.members = new Map();
    members.forEach(function (m) {
      var g = G.get(m.id), segs = [];
      memEls.get(m.id).forEach(function (e) {
        var dn = [d[3 * e.i], d[3 * e.i + 1], d[3 * e.i + 2], d[3 * e.j], d[3 * e.j + 1], d[3 * e.j + 2]], dl = new Float64Array(6), a, b;
        for (a = 0; a < 6; a++) { var s = 0; for (b = 0; b < 6; b++) s += e.T[a][b] * dn[b]; dl[a] = s; }
        if (e.cd.rec) e.cd.rec(dl);
        var f = new Float64Array(6);
        for (a = 0; a < 6; a++) { var s2 = e.f0[a]; for (b = 0; b < 6; b++) s2 += e.k[a][b] * dl[b]; f[a] = s2; }
        var L = e.L, Np = padd([-f[0]], pscale(pint(e.wx, 0), -1)), Sp = padd([f[1]], pint(e.wy, 0)), Mp = pint(Sp, -f[2]);
        var up = padd([dl[0], e.eps0], pscale(pint(Np, 0), 1 / e.EA));
        var thp = padd([dl[2], e.kap0], pscale(pint(Mp, 0), 1 / e.EI));
        var vp = pint(thp, dl[1]);
        segs.push({ xa: e.xa, xb: e.xa + L, L: L, N: Np, S: Sp, M: Mp, u: up, th: thp, v: vp, EA: e.EA, EI: e.EI, eps0: e.eps0, kap0: e.kap0 });
      });
      res.members.set(m.id, { L: g.L, c: g.c, s: g.s, segs: segs });
    });

    res.at = function (mid, x, side) {
      var mr = res.members.get(mid); if (!mr) return null;
      var segs = mr.segs, k = 0;
      x = Math.max(0, Math.min(mr.L, x));
      for (var q = 0; q < segs.length; q++) {
        if (x > segs[q].xb + 1e-9) continue;
        k = q; if (Math.abs(x - segs[q].xb) <= 1e-9 && side > 0 && q < segs.length - 1) k = q + 1;
        break;
      }
      var sg = segs[k], s = Math.max(0, Math.min(sg.L, x - sg.xa));
      var u = pval(sg.u, s), v = pval(sg.v, s);
      return { x: x, N: pval(sg.N, s), S: pval(sg.S, s), M: pval(sg.M, s), u: u, v: v, th: pval(sg.th, s), ux: u * mr.c - v * mr.s, uy: u * mr.s + v * mr.c, seg: sg };
    };
    res.ok = true;
    return res;
  }

  var GX = [0, -0.5384693101056831, 0.5384693101056831, -0.906179845938664, 0.906179845938664];
  var GW = [0.5688888888888889, 0.47862867049936647, 0.47862867049936647, 0.23692688505618908, 0.23692688505618908];

  // Maxwell–Mohr unit-load integral for the displacement at a node or at a station on a member.
  // dir: 'x' (unit force →), 'y' (unit force ↓), 'r' (unit moment ↺)
  function unitLoad(model, target, dir) {
    var ul = { type: 'point', Px: dir === 'x' ? 1 : 0, Py: dir === 'y' ? 1 : 0, M: dir === 'r' ? 1 : 0 };
    var extra = [];
    if (target.node != null) ul.node = target.node; else { ul.member = target.member; ul.t = target.t; extra.push({ member: target.member, t: target.t }); }
    var real = analyze(model, { extraSplits: extra });
    var virt = analyze(model, { loads: [ul], virtual: true, splitLoads: model.loads, extraSplits: extra });
    if (!real.ok || !virt.ok) return null;
    var rows = [], tot = { bend: 0, axial: 0, tAx: 0, tBend: 0, spring: 0, settle: 0 };
    model.members.forEach(function (m, idx) {
      var rs = real.members.get(m.id).segs, vs = virt.members.get(m.id).segs, row = { member: m.id, idx: idx, bend: 0, axial: 0, tAx: 0, tBend: 0 };
      for (var k = 0; k < rs.length; k++) {
        var a = rs[k], b = vs[k], h = a.L / 2;
        for (var g = 0; g < 5; g++) {
          var s = h * (1 + GX[g]), wt = GW[g] * h, M = pval(a.M, s), mm = pval(b.M, s), N = pval(a.N, s), nn = pval(b.N, s);
          row.bend += wt * M * mm / a.EI; row.axial += wt * N * nn / a.EA; row.tAx += wt * nn * a.eps0; row.tBend += wt * mm * a.kap0;
        }
      }
      rows.push(row); tot.bend += row.bend; tot.axial += row.axial; tot.tAx += row.tAx; tot.tBend += row.tBend;
    });
    var springRows = [], settleRows = [];
    real.springs.forEach(function (s, i) { var v = virt.springs[i]; var c = s.F * v.F / s.k; tot.spring += c; springRows.push({ node: s.node, comp: s.comp, F: s.F, f: v.F, k: s.k, c: c }); });
    real.supDofs.forEach(function (s, i) { if (!s.presc) return; var v = virt.supDofs[i]; var c = -v.R * s.presc; tot.settle += c; settleRows.push({ node: s.node, comp: s.comp, R: v.R, d: s.presc, c: c }); });
    var total = tot.bend + tot.axial + tot.tAx + tot.tBend + tot.spring + tot.settle, direct;
    if (target.node != null) { var dn = real.disp.get(target.node); direct = dir === 'x' ? dn.u : dir === 'y' ? -dn.v : dn.r; }
    else { var mr = real.members.get(target.member), p = real.at(target.member, target.t * mr.L, -1); direct = dir === 'x' ? p.ux : dir === 'y' ? -p.uy : p.th; }
    return { rows: rows, tot: tot, springRows: springRows, settleRows: settleRows, total: total, direct: direct, real: real, virt: virt };
  }

  // Influence line of a response for a unit load (↓ 1) travelling along every non-vertical member.
  // q: {kind:'reaction', node, comp:'x'|'y'|'m'} or {kind:'internal', member, t, what:'N'|'S'|'M'}
  function influence(model, q, nPer) {
    nPer = nPer || 20;
    var out = [], nById = new Map(model.nodes.map(function (n) { return [n.id, n]; }));
    var extra = q.kind === 'internal' ? [{ member: q.member, t: q.t }] : [];
    for (var mi = 0; mi < model.members.length; mi++) {
      var m = model.members[mi], a = nById.get(m.n1), b = nById.get(m.n2);
      if (Math.abs(b.x - a.x) < 1e-6 * Math.hypot(b.x - a.x, b.y - a.y)) continue;
      var ts = [], i;
      for (i = 0; i <= nPer; i++) ts.push(i / nPer);
      var atProbe = q.kind === 'internal' && q.member === m.id;
      if (atProbe) {
        ts = ts.filter(function (t) { return Math.abs(t - q.t) > 1e-6; });
        ts.push(q.t); ts.sort(function (p, r) { return p - r; });
      }
      var pts = [];
      for (i = 0; i < ts.length; i++) {
        var t = ts[i], ul = { type: 'point', Px: 0, Py: 1, M: 0 };
        if (t <= 0) ul.node = m.n1; else if (t >= 1) ul.node = m.n2; else { ul.member = m.id; ul.t = t; }
        var r = analyze(model, { loads: [ul], virtual: true, extraSplits: extra });
        if (!r.ok) return null;
        var val = 0;
        if (q.kind === 'reaction') {
          r.reactions.forEach(function (R) { if (R.node === q.node && !R.spring === !q.spring) val = q.comp === 'x' ? R.Rx : q.comp === 'y' ? R.Ry : R.M; });
        } else {
          var mr = r.members.get(q.member);
          if (atProbe && t === q.t && t > 0 && t < 1) {
            // load exactly at the section: left and right limits of the influence line
            pts.push({ t: t, val: r.at(q.member, q.t * mr.L, 1)[q.what] });
            val = r.at(q.member, q.t * mr.L, -1)[q.what];
          } else val = r.at(q.member, q.t * mr.L, -1)[q.what];
        }
        pts.push({ t: t, val: val });
      }
      out.push({ member: m.id, pts: pts });
    }
    return out;
  }

  // virtual work of system a (forces) on the deformations of system b
  function vwork(model, a, b) {
    var rows = [], total = 0;
    model.members.forEach(function (m) {
      var as = a.members.get(m.id).segs, bs = b.members.get(m.id).segs, r = 0;
      for (var k = 0; k < as.length; k++) {
        var A = as[k], B = bs[k], h = A.L / 2;
        for (var g = 0; g < 5; g++) {
          var s = h * (1 + GX[g]), wt = GW[g] * h, mm = pval(A.M, s), nn = pval(A.N, s);
          r += wt * (mm * pval(B.M, s) / B.EI + nn * pval(B.N, s) / B.EA + nn * B.eps0 + mm * B.kap0);
        }
      }
      rows.push(r); total += r;
    });
    var extra = 0;
    b.springs.forEach(function (sp, i) { extra += sp.F * a.springs[i].F / sp.k; });
    b.supDofs.forEach(function (sd, i) { if (sd.presc) extra -= a.supDofs[i].R * sd.presc; });
    return { rows: rows, extra: extra, total: total + extra };
  }

  // Force (flexibility) method: redundants X, primary structure, [δ]{X} + {Δ0} = {δs}  (Kirsch 3.2)
  function forceMethod(model, choice, full) {
    var n = determinacy(model).n, cands = [], out = { n: n };
    var rollers = [], others = [];
    model.nodes.forEach(function (nd) {
      var s = SUP[nd.sup]; if (!s) return;
      [1, 2, 0].forEach(function (c) { if (s[c]) (nd.sup === 'roller' ? rollers : others).push({ id: 'R:' + nd.id + ':' + c, kind: 'R', node: nd.id, comp: c }); });
    });
    cands = rollers.concat(others);
    model.members.forEach(function (m) {
      [0, 1].forEach(function (e) { if (!(m.rel || [])[e ? 5 : 2]) cands.push({ id: 'M:' + m.id + ':' + e, kind: 'M', member: m.id, end: e }); });
    });
    model.members.forEach(function (m) { var r = m.rel || []; if (r[2] && r[5] && !r[0] && !r[3]) cands.push({ id: 'N:' + m.id, kind: 'N', member: m.id }); });
    out.candidates = cands;
    var byId = {}; cands.forEach(function (c) { byId[c.id] = c; });
    function primary(list) {
      var p = JSON.parse(JSON.stringify(model));
      list.forEach(function (c) {
        if (c.kind === 'R') { var nd = p.nodes.find(function (q) { return q.id === c.node; }); nd.fix = (nd.fix || SUP[nd.sup]).slice(); nd.fix[c.comp] = 0; nd[['kx', 'ky', 'kr'][c.comp]] = 0; }
        else { var m = p.members.find(function (q) { return q.id === c.member; }); m.rel = (m.rel || [0, 0, 0, 0, 0, 0]).slice(); m.rel[c.kind === 'N' ? 0 : c.end ? 5 : 2] = 1; }
      });
      return p;
    }
    function valid(list) { var p = primary(list); return determinacy(p).n === n - list.length && analyze(p, { loads: [], virtual: true }).ok; }
    var chosen = null;
    if (choice && choice.length === n && choice.every(function (id) { return byId[id]; })) {
      chosen = choice.map(function (id) { return byId[id]; });
      if (new Set(choice).size !== n) { out.chosen = chosen; out.error = 'Choose a different redundant for each X.'; return out; }
      if (!valid(chosen)) { out.chosen = chosen; out.error = 'With this choice the primary structure is a mechanism (or one of the unknowns is not redundant). Pick another one.'; return out; }
    } else {
      chosen = [];
      for (var ci = 0; ci < cands.length && chosen.length < n; ci++) { var trial = chosen.concat([cands[ci]]); if (valid(trial)) chosen = trial; }
      if (chosen.length < n) { out.chosen = chosen; out.error = 'Could not choose the redundants automatically for this structure.'; return out; }
    }
    out.chosen = chosen;
    var prim = primary(chosen); out.primary = prim;
    out.real0 = analyze(prim);
    out.virt = chosen.map(function (c) {
      var o = { loads: [], virtual: true, splitLoads: model.loads };
      if (c.kind === 'R') {
        var nd = model.nodes.find(function (q) { return q.id === c.node; }), b = (nd.ang || 0) * Math.PI / 180;
        o.loads = [{ type: 'point', node: c.node, Px: c.comp === 0 ? Math.cos(b) : c.comp === 1 ? -Math.sin(b) : 0, Py: c.comp === 0 ? -Math.sin(b) : c.comp === 1 ? -Math.cos(b) : 0, M: c.comp === 2 ? 1 : 0 }];
      } else if (c.kind === 'M') o.endForces = [{ member: c.member, dof: c.end ? 5 : 2, value: c.end ? 1 : -1 }];
      else o.endForces = [{ member: c.member, dof: 0, value: -1 }];
      return analyze(prim, o);
    });
    if (!out.real0.ok || out.virt.some(function (v) { return !v.ok; })) { out.error = 'The primary structure could not be analysed.'; return out; }
    var i, j;
    out.D0 = []; out.D0rows = []; out.delta = []; out.deltaRows = []; out.rhs = [];
    for (i = 0; i < n; i++) {
      var w0 = vwork(model, out.virt[i], out.real0); out.D0.push(w0.total); out.D0rows.push(w0.rows);
      out.delta.push([]); out.deltaRows.push([]);
      for (j = 0; j < n; j++) { var w = vwork(model, out.virt[i], out.virt[j]); out.delta[i].push(w.total); out.deltaRows[i].push(w.rows); }
      var c = chosen[i], pres = 0;
      if (c.kind === 'R') { var nd2 = model.nodes.find(function (q) { return q.id === c.node; }); pres = +nd2[['dx', 'dy', 'dr'][c.comp]] || 0; }
      out.rhs.push(pres);
    }
    var inv = invert(out.delta); if (!inv) { out.error = 'The flexibility matrix is singular.'; return out; }
    out.X = out.rhs.map(function (_, a) { var s = 0; for (var b2 = 0; b2 < n; b2++) s += inv[a][b2] * (out.rhs[b2] - out.D0[b2]); return s; });
    if (full && full.ok) out.check = chosen.map(function (c) {
      if (c.kind === 'R') { var sd = full.supDofs.find(function (q) { return q.node === c.node && q.comp === c.comp; }); return sd ? sd.R : NaN; }
      var mr = full.members.get(c.member);
      return c.kind === 'M' ? full.at(c.member, c.end ? mr.L : 0, c.end ? -1 : 1).M : full.at(c.member, 0, 1).N;
    });
    return out;
  }

  var api = { compileExpr: compileExpr, distQ: distQ, analyze: analyze, forceMethod: forceMethod, unitLoad: unitLoad, influence: influence, determinacy: determinacy, pval: pval, pshift: pshift, SUP: SUP };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.Solver = api;
})(typeof window !== 'undefined' ? window : this);
