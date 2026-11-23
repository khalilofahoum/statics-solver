const So = require('../solver.js');
const Cl = require('../classic.js');
const N = (id, x, y, sup, o) => Object.assign({ id, x, y, sup: sup || 'none', ang: 0 }, o || {});
const M = (id, n1, n2, o) => Object.assign({ id, n1, n2, E: 1, A: 1, I: 1, rel: [0, 0, 0, 0, 0, 0] }, o || {});
const q = (m, v) => ({ type: 'dist', member: m, t1: 0, t2: 1, q1: v || 1, q2: v || 1, dir: 'Y' });
const f = v => Math.abs(v) < 1e-9 ? 0 : +v.toFixed(4);
function run(name, mod, expect) {
  const r = Cl.analyze(mod, So);
  if (!r.ok) { console.log(expect === 'na' ? 'PASS' : 'FAIL', name, 'not applicable:', r.reasons.join('; ')); return; }
  const ok = r.maxDiff < 1e-6 * (1 + r.scale);
  console.log(ok ? 'PASS' : 'FAIL', name, '| θ joints', r.thetaNodes.length, 'sway', r.modes.length, 'stubs', r.stubs.length, '| cycles', r.md.case0.cycles, '| maxDiff', r.maxDiff.toExponential(2),
    '| M:', r.members.map(m => `[${f(m.Msd[0])},${f(m.Msd[1])}]`).join(' '), expect ? '| expect ' + expect : '');
  if (!ok) console.log('   md:', r.members.map(m => `[${f(m.Mmd[0])},${f(m.Mmd[1])}]`).join(' '), ' ref:', r.members.map(m => `[${f(m.Mref[0])},${f(m.Mref[1])}]`).join(' '));
}
// 1 propped cantilever: M_A = qL²/8 = 12.5 hogging (member-end CCW at A = +12.5), L=10
run('propped cantilever', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 10, 0, 'roller')], members: [M(1, 1, 2)], loads: [q(1)] }, 'no unknowns → na');
// 2 two-span continuous beam, equal spans L=6, q=1: M_B = qL²/8 = 4.5
run('two-span beam', { settings: {}, nodes: [N(1, 0, 0, 'pin'), N(2, 6, 0, 'roller'), N(3, 12, 0, 'roller')], members: [M(1, 1, 2), M(2, 2, 3)], loads: [q(1), q(2)] }, 'M_B = 4.5');
// 3 three-span with fixed end, point load, different I
run('three-span fixed end', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 5, 0, 'roller'), N(3, 11, 0, 'roller'), N(4, 15, 0, 'roller')], members: [M(1, 1, 2), M(2, 2, 3, { I: 2 }), M(3, 3, 4)], loads: [q(1, 15), q(2, 15), q(3, 15), { type: 'point', member: 2, t: .5, Px: 0, Py: 40, M: 0 }] });
// 4 fixed portal with lateral load: sway
run('portal fixed, lateral', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 0, 4), N(3, 6, 4), N(4, 6, 0, 'fixed')], members: [M(1, 1, 2), M(2, 2, 3, { I: 2 }), M(3, 3, 4)], loads: [{ type: 'point', node: 2, Px: 10, Py: 0, M: 0 }] }, 'antisymmetric: column bases equal');
// 5 portal pinned bases, vertical q + lateral, unequal columns
run('portal pinned, unequal legs', { settings: {}, nodes: [N(1, 0, 0, 'pin'), N(2, 0, 4), N(3, 6, 4), N(4, 6, 1, 'pin')], members: [M(1, 1, 2), M(2, 2, 3), M(3, 3, 4)], loads: [q(2, 3), { type: 'point', node: 2, Px: 5, Py: 0, M: 0 }, { type: 'dist', member: 1, t1: 0, t2: 1, q1: 2, q2: 2, dir: 'X' }] });
// 6 frame with an inclined leg (sway couples u and v)
run('inclined leg frame', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 2, 4), N(3, 7, 4), N(4, 7, 0, 'fixed')], members: [M(1, 1, 2), M(2, 2, 3), M(3, 3, 4)], loads: [q(2, 4), { type: 'point', member: 1, t: .5, Px: 6, Py: 0, M: 0 }] });
// 7 beam with overhang (stub) and a joint moment
run('overhang + joint moment', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 6, 0, 'roller'), N(3, 12, 0, 'roller'), N(4, 14, 0)], members: [M(1, 1, 2), M(2, 2, 3), M(3, 3, 4)], loads: [q(1, 2), q(3, 5), { type: 'point', node: 4, Px: 0, Py: 7, M: 0 }, { type: 'point', node: 2, Px: 0, Py: 0, M: 9 }] });
// 8 two-storey frame, 2 sway modes, hinge in a beam, parabolic load
run('two-storey frame', {
  settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 0, 3), N(3, 0, 6), N(4, 5, 6), N(5, 5, 3), N(6, 5, 0, 'pin')],
  members: [M(1, 1, 2), M(2, 2, 3), M(3, 3, 4, { rel: [0, 0, 0, 0, 0, 1] }), M(4, 4, 5), M(5, 5, 6), M(6, 2, 5, { I: 3 })],
  loads: [{ type: 'point', node: 3, Px: 8, Py: 0, M: 0 }, { type: 'point', node: 2, Px: 4, Py: 0, M: 0 }, { type: 'dist', member: 6, t1: 0, t2: 1, q1: 0, qm: 6, q2: 0, shape: 'parabola', dir: 'Y' }, q(3, 2)]
});
// 9 beam with sliding clamp (vertical sway) and partial triangular load
run('guide support beam', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 4, 0, 'roller'), N(3, 9, 0, 'guide', { ang: 90 })], members: [M(1, 1, 2), M(2, 2, 3)], loads: [{ type: 'dist', member: 2, t1: .2, t2: .9, q1: 0, q2: 6, dir: 'Y' }, q(1, 3)] });
// 10 inclined roller frame with point moment on a member
run('inclined roller', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 0, 3), N(3, 4, 3, 'roller', { ang: 30 })], members: [M(1, 1, 2), M(2, 2, 3)], loads: [{ type: 'point', member: 2, t: .4, Px: 0, Py: 5, M: 3 }, { type: 'point', node: 2, Px: 2, Py: 0, M: 0 }] });
// not applicable cases
run('truss', { settings: {}, nodes: [N(1, 0, 0, 'pin'), N(2, 4, 0, 'roller'), N(3, 2, 2)], members: [M(1, 1, 2, { rel: [0, 0, 1, 0, 0, 1] }), M(2, 1, 3, { rel: [0, 0, 1, 0, 0, 1] }), M(3, 2, 3, { rel: [0, 0, 1, 0, 0, 1] })], loads: [] }, 'na');
run('spring', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 4, 0, 'none', { ky: 3 }), N(3, 8, 0, 'roller')], members: [M(1, 1, 2), M(2, 2, 3)], loads: [q(1)] }, 'na');
