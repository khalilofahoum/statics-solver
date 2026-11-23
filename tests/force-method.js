const So = require('../solver.js');
const N = (id, x, y, sup, o) => Object.assign({ id, x, y, sup: sup || 'none', ang: 0 }, o || {});
const M = (id, n1, n2, o) => Object.assign({ id, n1, n2, E: 1, A: 1, I: 1, rel: [0, 0, 0, 0, 0, 0] }, o || {});
const f = v => Math.abs(v) < 1e-8 ? 0 : +v.toFixed(5);
function run(name, mod, choice) {
  const full = So.analyze(mod); const F = So.forceMethod(mod, choice, full);
  if (F.error) { console.log(name, 'ERR', F.error, F.chosen.map(c => c.id)); return; }
  const ok = F.X.every((x, i) => Math.abs(x - F.check[i]) < 1e-6 * (1 + Math.abs(x)));
  console.log(ok ? 'PASS' : 'FAIL', name, 'n=', F.n, F.chosen.map(c => c.id).join(','), 'X=', F.X.map(f), 'check=', F.check.map(f), 'D0', F.D0.map(f), 'delta', JSON.stringify(F.delta.map(r => r.map(f))));
}
const q = m => ({ type: 'dist', member: m, t1: 0, t2: 1, q1: 1, q2: 1, dir: 'Y' });
const propped = () => ({ settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 10, 0, 'roller')], members: [M(1, 1, 2)], loads: [q(1)] });
run('propped', propped());
run('propped-MA', propped(), ['R:1:2']);
run('propped-Mint', propped(), ['M:1:0']);
const cont = () => ({ settings: {}, nodes: [N(1, 0, 0, 'pin'), N(2, 5, 0, 'roller'), N(3, 11, 0, 'roller'), N(4, 15, 0, 'roller')], members: [M(1, 1, 2), M(2, 2, 3), M(3, 3, 4)], loads: [q(1), q(2), q(3), { type: 'point', member: 2, t: .5, Py: 40 }] });
run('cont', cont());
run('cont-3M', cont(), ['M:1:1', 'M:2:1']);
run('fixfix', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 6, 0, 'fixed')], members: [M(1, 1, 2)], loads: [q(1), { type: 'point', member: 1, t: .3, Px: 2, Py: 3 }] });
run('portal-fixed', { settings: { axialRigid: true }, nodes: [N(1, 0, 0, 'fixed'), N(2, 0, 3), N(3, 4, 3), N(4, 4, 0, 'fixed', { dy: -0.01 })], members: [M(1, 1, 2), M(2, 2, 3, { alpha: 1e-5, Tt: 20, Tb: 0, h: .3 }), M(3, 3, 4)], loads: [{ type: 'point', node: 2, Px: 10 }] });
const hb = () => ({ rel: [0, 0, 1, 0, 0, 1] });
run('truss-x', { settings: {}, nodes: [N(1, 0, 0, 'pin'), N(2, 3, 0, 'roller'), N(3, 3, 3), N(4, 0, 3)], members: [M(1, 1, 2, hb()), M(2, 2, 3, hb()), M(3, 3, 4, hb()), M(4, 4, 1, hb()), M(5, 1, 3, hb()), M(6, 2, 4, hb())], loads: [{ type: 'point', node: 3, Px: 10, Py: 5 }] });
run('incl+spring', { settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, 4, 0, 'none', { ky: 3 }), N(3, 8, 0, 'roller', { ang: 30 })], members: [M(1, 1, 2), M(2, 2, 3)], loads: [q(1), q(2)] });
run('bad', propped(), ['R:1:0']);
