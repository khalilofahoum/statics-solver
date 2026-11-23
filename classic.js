/* Classical displacement methods for beams and frames (Kirsch 3.3):
 *   slope-deflection (joint rotations θ and sway displacements Δ as unknowns) and
 *   moment distribution (Cross), with separate sway cases combined through the sway equations.
 * Assumptions: bending deformation only (inextensible members), loads only.
 * Sign convention inside this file: end moments M_ij (acting on the member end), joint rotations θ and
 * chord rotations ψ are all positive counter-clockwise.
 */
(function (root) {
  'use strict';

  function gauss(A, b) {
    var n = b.length, i, j, k, M = A.map(function (r, q) { return r.concat([b[q]]); }), mx = 0;
    for (i = 0; i < n; i++) for (j = 0; j < n; j++) mx = Math.max(mx, Math.abs(M[i][j]));
    for (k = 0; k < n; k++) {
      var p = k; for (i = k + 1; i < n; i++) if (Math.abs(M[i][k]) > Math.abs(M[p][k])) p = i;
      if (Math.abs(M[p][k]) <= 1e-11 * mx) return null;
      var t = M[k]; M[k] = M[p]; M[p] = t;
      for (i = k + 1; i < n; i++) { var f = M[i][k] / M[k][k]; if (f) for (j = k; j <= n; j++) M[i][j] -= f * M[k][j]; }
    }
    var x = new Array(n);
    for (i = n - 1; i >= 0; i--) { var s = M[i][n]; for (j = i + 1; j < n; j++) s -= M[i][j] * x[j]; x[i] = s / M[i][i]; }
    return x;
  }
  // null space of C (rows of length ncol) by reduced row echelon form, pivoting along the given column order
  function nullspace(C, ncol, order) {
    var R = C.map(function (r) { return r.slice(); }), piv = [], r = 0, isPiv = {};
    order.forEach(function (col) {
      if (r >= R.length) return;
      var p = r, i, j; for (i = r + 1; i < R.length; i++) if (Math.abs(R[i][col]) > Math.abs(R[p][col])) p = i;
      if (Math.abs(R[p][col]) < 1e-9) return;
      var t = R[r]; R[r] = R[p]; R[p] = t;
      var d = R[r][col]; for (j = 0; j < ncol; j++) R[r][j] /= d;
      for (i = 0; i < R.length; i++) if (i !== r) { var f = R[i][col]; if (f) for (j = 0; j < ncol; j++) R[i][j] -= f * R[r][j]; }
      piv.push(col); isPiv[col] = 1; r++;
    });
    return order.filter(function (c) { return !isPiv[c]; }).map(function (fc) {
      var x = new Array(ncol).fill(0); x[fc] = 1;
      piv.forEach(function (pc, k) { x[pc] = -R[k][fc]; });
      return { free: fc, vec: x };
    });
  }
  function gdir(dir, c, s) {
    switch (dir) { case 'X': return [1, 0]; case 'proj': return [0, -Math.abs(c)]; case 'perp': return [s, -c]; case 'axial': return [c, s]; default: return [0, -1]; }
  }

  function analyze(model, Solver) {
    var SUP = Solver.SUP, out = { ok: false, reasons: [] }, nodes = model.nodes, N = nodes.length;
    var idx = {}; nodes.forEach(function (n, i) { idx[n.id] = i; });
    // ---- applicability
    if (model.members.some(function (m) { var r = m.rel || []; return r[0] || r[1] || r[3] || r[4]; })) out.reasons.push('a member has a shear or axial slider');
    if (nodes.some(function (n) { var s = SUP[n.sup] || [0, 0, 0]; return (n.kx > 0 && !s[0]) || (n.ky > 0 && !s[1]) || (n.kr > 0 && !s[2]); })) out.reasons.push('the structure has elastic springs');
    if (nodes.some(function (n) { var s = SUP[n.sup]; return s && ((s[0] && +n.dx) || (s[1] && +n.dy) || (s[2] && +n.dr)); })) out.reasons.push('a support settlement is prescribed');
    if (model.members.some(function (m) { return (+m.alpha && (+m.Tt || +m.Tb)) || +m.dL; })) out.reasons.push('a member has a temperature change or lack of fit');
    if (model.members.every(function (m) { var r = m.rel || []; return r[2] && r[5]; })) out.reasons.push('all members are pin-ended bars (a truss has no bending unknowns)');
    if (out.reasons.length) return out;

    // ---- loads: joint loads and loads along members
    var JL = nodes.map(function () { return { Fx: 0, Fy: 0, M: 0 }; }), memLoads = {};
    (model.loads || []).forEach(function (l) {
      var m = l.member != null ? model.members.find(function (q) { return q.id === l.member; }) : null;
      if (l.type === 'point') {
        var ni = l.node != null ? idx[l.node] : !m ? null : l.t <= 1e-9 ? idx[m.n1] : l.t >= 1 - 1e-9 ? idx[m.n2] : null;
        if (ni != null) { JL[ni].Fx += +l.Px || 0; JL[ni].Fy -= +l.Py || 0; JL[ni].M += +l.M || 0; return; }
      }
      if (m) (memLoads[m.id] || (memLoads[m.id] = [])).push(l);
    });

    // ---- cantilever stubs are statically determinate: replace each by its root forces on the joint
    var active = model.members.slice(); out.stubs = [];
    for (var guard = 0; guard < 1000; guard++) {
      var found = null;
      for (var ni0 = 0; ni0 < N && !found; ni0++) {
        var nd0 = nodes[ni0]; if (SUP[nd0.sup]) continue;
        var touch = active.filter(function (m) { return m.n1 === nd0.id || m.n2 === nd0.id; });
        if (touch.length === 1 && active.length > 1 && !(touch[0].rel || []).some(Boolean)) found = { tip: ni0, m: touch[0] };
      }
      if (!found) break;
      var sm = found.m, tipFirst = sm.n1 === nodes[found.tip].id, rootI = idx[tipFirst ? sm.n2 : sm.n1], tp = nodes[found.tip], rt = nodes[rootI];
      var iso = {
        settings: {}, nodes: [{ id: 1, x: rt.x, y: rt.y, sup: 'fixed', ang: 0 }, { id: 2, x: tp.x, y: tp.y, sup: 'none', ang: 0 }],
        members: [{ id: 1, n1: tipFirst ? 2 : 1, n2: tipFirst ? 1 : 2, E: sm.E, A: sm.A, I: sm.I, rel: [0, 0, 0, 0, 0, 0] }],
        loads: (memLoads[sm.id] || []).map(function (l) { return Object.assign({}, l, { member: 1 }); }).concat([{ type: 'point', node: 2, Px: JL[found.tip].Fx, Py: -JL[found.tip].Fy, M: JL[found.tip].M }])
      };
      var rs = Solver.analyze(iso); if (!rs.ok) break;
      var Rr = rs.reactions[0];
      JL[rootI].Fx -= Rr.Rx; JL[rootI].Fy -= Rr.Ry; JL[rootI].M -= Rr.M;
      JL[found.tip] = { Fx: 0, Fy: 0, M: 0 }; delete memLoads[sm.id];
      out.stubs.push({ member: sm.id, root: rt.id, tip: tp.id, M: Rr.M, Fx: -Rr.Rx, Fy: -Rr.Ry });
      active = active.filter(function (m) { return m !== sm; });
    }

    // ---- members, real and effective hinges
    var ms = active.map(function (m) {
      var a = nodes[idx[m.n1]], b = nodes[idx[m.n2]], dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy), r = m.rel || [];
      return { id: m.id, src: m, n: [idx[m.n1], idx[m.n2]], L: L, c: dx / L, s: dy / L, EI: m.E * m.I, k: m.E * m.I / L, h: [!!r[2], !!r[5]], eff: [false, false], F: [0, 0], psi: [] };
    });
    var rotFix = nodes.map(function (n) { var s = SUP[n.sup]; return !!(s && s[2]); });
    var tIdx = new Array(N).fill(-1), thetaNodes = [];
    nodes.forEach(function (n, ni) {
      var ends = []; ms.forEach(function (m) { [0, 1].forEach(function (e) { if (m.n[e] === ni && !m.h[e]) ends.push([m, e]); }); });
      if (rotFix[ni] || !ends.length) return;
      // a simple end support (one rigid member end, free to rotate, no applied moment) behaves as a hinge
      if (ends.length === 1 && Math.abs(JL[ni].M) < 1e-12) { ends[0][0].h[ends[0][1]] = true; ends[0][0].eff[ends[0][1]] = true; return; }
      tIdx[ni] = thetaNodes.length; thetaNodes.push(ni);
    });

    // ---- independent joint translations (sway) with inextensible members
    var C = [], used = new Array(N).fill(false);
    ms.forEach(function (m) { var r = new Array(2 * N).fill(0); r[2 * m.n[0]] = -m.c; r[2 * m.n[0] + 1] = -m.s; r[2 * m.n[1]] = m.c; r[2 * m.n[1] + 1] = m.s; C.push(r); used[m.n[0]] = used[m.n[1]] = true; });
    nodes.forEach(function (n, ni) {
      var s = SUP[n.sup], row;
      if (!used[ni] || (s && s[0] && s[1])) { row = new Array(2 * N).fill(0); row[2 * ni] = 1; C.push(row); row = new Array(2 * N).fill(0); row[2 * ni + 1] = 1; C.push(row); }
      else if (s && s[1]) { var b = (n.ang || 0) * Math.PI / 180; row = new Array(2 * N).fill(0); row[2 * ni] = -Math.sin(b); row[2 * ni + 1] = Math.cos(b); C.push(row); }
    });
    var order = [], i, d;
    for (i = 0; i < N; i++) order.push(2 * i + 1);
    for (i = N - 1; i >= 0; i--) order.push(2 * i);
    var modes = nullspace(C, 2 * N, order).map(function (b) {
      return { node: nodes[(b.free - b.free % 2) / 2].id, dir: b.free % 2 ? 'v' : 'u', vec: b.vec.map(function (v) { return Math.abs(v) < 1e-12 ? 0 : v; }) };
    });
    var S = modes.length, nt = thetaNodes.length, nq = nt + S;
    ms.forEach(function (m) {
      m.psi = modes.map(function (md) { var v = md.vec; return ((v[2 * m.n[1] + 1] - v[2 * m.n[0] + 1]) * m.c - (v[2 * m.n[1]] - v[2 * m.n[0]]) * m.s) / m.L; });
      m.psi = m.psi.map(function (p) { return Math.abs(p) < 1e-12 ? 0 : p; });
    });

    // ---- fixed-end moments (member isolated between immovable joints; hinged ends stay hinged)
    ms.forEach(function (m) {
      var ll = memLoads[m.id]; if (!ll || (m.h[0] && m.h[1])) return;
      var a = nodes[m.n[0]], b = nodes[m.n[1]];
      var r = Solver.analyze({
        settings: {}, nodes: [{ id: 1, x: a.x, y: a.y, sup: 'fixed', ang: 0 }, { id: 2, x: b.x, y: b.y, sup: 'fixed', ang: 0 }],
        members: [{ id: 1, n1: 1, n2: 2, E: m.src.E, A: m.src.A, I: m.src.I, rel: [0, 0, m.h[0] ? 1 : 0, 0, 0, m.h[1] ? 1 : 0] }],
        loads: ll.map(function (l) { return Object.assign({}, l, { member: 1 }); })
      });
      if (!r.ok) return;
      var f0 = -r.at(1, 0, 1).M, f1 = r.at(1, m.L, -1).M, sc = Math.max(Math.abs(f0), Math.abs(f1), 1e-30);
      m.F = [Math.abs(f0) < 1e-10 * sc || m.h[0] ? 0 : f0, Math.abs(f1) < 1e-10 * sc || m.h[1] ? 0 : f1];
    });

    // ---- virtual work of the loads in each sway mode
    var W = modes.map(function (md, di) {
      var v = md.vec, w = 0;
      JL.forEach(function (j, ni) { w += j.Fx * v[2 * ni] + j.Fy * v[2 * ni + 1]; });
      ms.forEach(function (m) {
        var di0 = [v[2 * m.n[0]], v[2 * m.n[0] + 1]], dj = [v[2 * m.n[1]], v[2 * m.n[1] + 1]];
        var disp = function (t) { return [(1 - t) * di0[0] + t * dj[0], (1 - t) * di0[1] + t * dj[1]]; };
        (memLoads[m.id] || []).forEach(function (l) {
          if (l.type === 'point') { var p = disp(l.t); w += (+l.Px || 0) * p[0] - (+l.Py || 0) * p[1] + (+l.M || 0) * m.psi[di]; return; }
          var lo = Math.min(l.t1, l.t2), hi = Math.max(l.t1, l.t2), span = (hi - lo) * m.L, g = gdir(l.dir, m.c, m.s), nS = 60, sum = 0;
          if (span < 1e-12) return;
          for (var q = 0; q <= nS; q++) { var tau = q / nS, pp = disp(lo + tau * (hi - lo)); sum += (q === 0 || q === nS ? 1 : q % 2 ? 4 : 2) * Solver.distQ(l, tau, span) * (g[0] * pp[0] + g[1] * pp[1]); }
          w += sum * span / nS / 3;
        });
      });
      return Math.abs(w) < 1e-12 ? 0 : w;
    });

    // ---- slope-deflection: end moments as linear functions of the unknowns
    function endExpr(m, e) {
      var co = new Array(nq).fill(0), o = 1 - e;
      if (m.h[e]) return { member: m.id, e: e, coef: co, c0: 0 };
      var ti = tIdx[m.n[e]], tj = tIdx[m.n[o]], f = m.h[o] ? 3 * m.k : 2 * m.k;
      if (m.h[o]) { if (ti >= 0) co[ti] += f; for (d = 0; d < S; d++) co[nt + d] -= f * m.psi[d]; }
      else { if (ti >= 0) co[ti] += 2 * f; if (tj >= 0) co[tj] += f; for (d = 0; d < S; d++) co[nt + d] -= 3 * f * m.psi[d]; }
      return { member: m.id, e: e, coef: co, c0: m.F[e] };
    }
    var exprs = []; ms.forEach(function (m) { m.ex = [endExpr(m, 0), endExpr(m, 1)]; exprs.push(m.ex[0], m.ex[1]); });
    var A = [], bvec = [], eqs = [];
    thetaNodes.forEach(function (ni) {
      var row = new Array(nq).fill(0), c0 = 0, ends = [];
      ms.forEach(function (m) { [0, 1].forEach(function (e) { if (m.n[e] !== ni || m.h[e]) return; ends.push({ member: m.id, e: e }); m.ex[e].coef.forEach(function (v, q) { row[q] += v; }); c0 += m.ex[e].c0; }); });
      A.push(row); bvec.push(JL[ni].M - c0); eqs.push({ kind: 'joint', node: nodes[ni].id, ends: ends, row: row, c0: c0, rhs: JL[ni].M });
    });
    modes.forEach(function (md, di) {
      var row = new Array(nq).fill(0), c0 = 0;
      ms.forEach(function (m) { if (!m.psi[di]) return; [0, 1].forEach(function (e) { m.ex[e].coef.forEach(function (v, q) { row[q] += v * m.psi[di]; }); c0 += m.ex[e].c0 * m.psi[di]; }); });
      A.push(row); bvec.push(-W[di] - c0); eqs.push({ kind: 'sway', mode: di, row: row, c0: c0 + W[di], rhs: 0 });
    });
    var q = gauss(A, bvec);
    if (!q) { out.reasons.push('the slope-deflection equations are singular (the structure is unstable without axial deformation)'); return out; }
    ms.forEach(function (m) { m.Msd = m.ex.map(function (x) { return x.coef.reduce(function (s, v, k) { return s + v * q[k]; }, x.c0); }); });

    // ---- moment distribution
    var ends = [];
    ms.forEach(function (m, mi) { [0, 1].forEach(function (e) { ends.push({ mi: mi, member: m.id, e: e, ni: m.n[e], node: nodes[m.n[e]].id, joint: tIdx[m.n[e]] >= 0, S: m.h[e] ? 0 : m.h[1 - e] ? 3 * m.k : 4 * m.k, co: !m.h[e] && !m.h[1 - e] ? 0.5 : 0, DF: 0 }); }); });
    thetaNodes.forEach(function (ni) { var tot = 0; ends.forEach(function (en) { if (en.ni === ni) tot += en.S; }); ends.forEach(function (en) { if (en.ni === ni) en.DF = tot ? en.S / tot : 0; }); });
    function distribute(F0, Mext) {
      var rows = [{ type: 'FEM', vals: F0.slice() }], total = F0.slice(), U = {}, sc = 1e-30, cycles = 0;
      F0.forEach(function (v) { sc = Math.max(sc, Math.abs(v)); });
      thetaNodes.forEach(function (ni) { var s = Mext ? -Mext[ni] : 0; if (Mext) sc = Math.max(sc, Math.abs(Mext[ni])); ends.forEach(function (en, k) { if (en.ni === ni) s += F0[k]; }); U[ni] = s; });
      for (var cyc = 0; cyc < 400; cyc++) {
        var bal = ends.map(function (en) { return en.joint ? -U[en.ni] * en.DF : 0; }), co = new Array(ends.length).fill(0), mxb = 0;
        bal.forEach(function (v, k) { mxb = Math.max(mxb, Math.abs(v)); if (ends[k].co) co[k % 2 ? k - 1 : k + 1] += v * ends[k].co; });
        if (mxb < 1e-13 * sc) break;
        cycles++; rows.push({ type: 'Bal', vals: bal }, { type: 'CO', vals: co });
        bal.forEach(function (v, k) { total[k] += v + co[k]; });
        thetaNodes.forEach(function (ni) { var s = 0; ends.forEach(function (en, k) { if (en.ni === ni) s += co[k]; }); U[ni] = s; });
      }
      return { rows: rows, total: total, cycles: cycles };
    }
    var Rof = function (Mv, di) { var s = 0; ms.forEach(function (m, mi) { s += m.psi[di] * (Mv[2 * mi] + Mv[2 * mi + 1]); }); return s; };
    var F0 = []; ms.forEach(function (m) { F0.push(m.F[0], m.F[1]); });
    var Mext = JL.map(function (j) { return j.M; });
    var md = { ends: ends, case0: distribute(F0, Mext), sway: [], c: [] };
    modes.forEach(function (mo, di) {
      var Fs = []; ms.forEach(function (m) { [0, 1].forEach(function (e) { Fs.push(m.h[e] ? 0 : (m.h[1 - e] ? -3 : -6) * m.k * m.psi[di]); }); });
      md.sway.push(distribute(Fs, null));
    });
    if (S) {
      md.Rmat = modes.map(function (_, di) { return md.sway.map(function (sw) { return Rof(sw.total, di); }); });
      md.R0 = modes.map(function (_, di) { return Rof(md.case0.total, di); });
      md.c = gauss(md.Rmat, md.R0.map(function (r, di) { return -(r + W[di]); }));
      if (!md.c) { out.reasons.push('the sway equations of the moment distribution are singular'); return out; }
    }
    md.final = md.case0.total.map(function (v, k) { return md.sway.reduce(function (s, sw, di) { return s + md.c[di] * sw.total[k]; }, v); });

    // ---- check against the matrix solver (same assumption: no axial deformation)
    var clone = JSON.parse(JSON.stringify(model)); clone.settings = Object.assign({}, clone.settings, { axialRigid: true });
    var ref = Solver.analyze(clone), maxDiff = 0, scale = 1e-30;
    ms.forEach(function (m, mi) {
      m.Mmd = [md.final[2 * mi], md.final[2 * mi + 1]];
      m.Mref = ref.ok ? [-ref.at(m.id, 0, 1).M, ref.at(m.id, m.L, -1).M] : [NaN, NaN];
      [0, 1].forEach(function (e) { scale = Math.max(scale, Math.abs(m.Mref[e])); maxDiff = Math.max(maxDiff, Math.abs(m.Msd[e] - m.Mref[e]), Math.abs(m.Mmd[e] - m.Mref[e])); });
    });
    out.ok = true; out.maxDiff = maxDiff; out.scale = scale;
    out.thetaNodes = thetaNodes.map(function (ni) { return nodes[ni].id; }); out.modes = modes; out.W = W; out.JL = JL;
    out.members = ms.map(function (m) { return { id: m.id, n: [nodes[m.n[0]].id, nodes[m.n[1]].id], L: m.L, k: m.k, EI: m.EI, h: m.h, eff: m.eff, psi: m.psi, F: m.F, ex: m.ex, Msd: m.Msd, Mmd: m.Mmd, Mref: m.Mref }; });
    out.eqs = eqs; out.A = A; out.b = bvec; out.q = q; out.md = md; out.nt = nt;
    return out;
  }

  var api = { analyze: analyze };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.Classic = api;
})(typeof window !== 'undefined' ? window : this);
