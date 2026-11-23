const So = require('../solver.js');
const N = (id, x, y, sup, o) => Object.assign({ id, x, y, sup: sup || 'none', ang: 0 }, o || {});
const M = (id, n1, n2, o) => Object.assign({ id, n1, n2, E: 1, A: 1, I: 1, rel: [0, 0, 0, 0, 0, 0] }, o || {});
const chk = (name, got, exp, tol) => console.log(Math.abs(got - exp) <= (tol || 1e-6) * (1 + Math.abs(exp)) ? 'PASS' : 'FAIL', name, got, 'expected', exp);
const beam = (L, loads, sup2) => ({ settings: {}, nodes: [N(1, 0, 0, sup2 ? 'fixed' : 'pin'), N(2, L, 0, sup2 || 'roller')], members: [M(1, 1, 2)], loads });
const L = 6, q = 4;
// parabola 0 - q - 0 on simply supported beam: total = 2qL/3, R = qL/3, Mmid = 5qL^2/48
let r = So.analyze(beam(L, [{ type: 'dist', member: 1, t1: 0, t2: 1, q1: 0, qm: q, q2: 0, shape: 'parabola', dir: 'Y' }]));
chk('parabola R', r.reactions[0].Ry, q * L / 3); chk('parabola Mmid', r.at(1, L / 2).M, 5 * q * L * L / 48);
// same as a function
r = So.analyze(beam(L, [{ type: 'dist', member: 1, t1: 0, t2: 1, shape: 'func', expr: '4*q0*x*(L-x)/L^2'.replace('q0', q), dir: 'Y' }]));
chk('func parabola R', r.reactions[0].Ry, q * L / 3); chk('func parabola Mmid', r.at(1, L / 2).M, 5 * q * L * L / 48);
// sine load q sin(pi x/L): R = qL/pi, Mmid = qL^2/pi^2, vmid = qL^4/(pi^4 EI)
r = So.analyze(beam(L, [{ type: 'dist', member: 1, t1: 0, t2: 1, shape: 'func', expr: '4 sin(pi x / L)', dir: 'Y' }]));
chk('sine R', r.reactions[0].Ry, q * L / Math.PI, 1e-5); chk('sine Mmid', r.at(1, L / 2).M, q * L * L / Math.PI ** 2, 1e-5); chk('sine vmid', -r.at(1, L / 2).v, q * L ** 4 / Math.PI ** 4, 1e-5);
// half parabola on cantilever (q = q0 (x/L)^2 from free end... here x from the clamp): total = qL/3, M_clamp = -q L^2/4
r = So.analyze({ settings: {}, nodes: [N(1, 0, 0, 'fixed'), N(2, L, 0)], members: [M(1, 1, 2)], loads: [{ type: 'dist', member: 1, t1: 0, t2: 1, q1: 0, qm: q / 4, q2: q, shape: 'parabola', dir: 'Y' }] });
chk('half-parabola R', r.reactions[0].Ry, q * L / 3); chk('half-parabola M clamp', r.at(1, 0, 1).M, -q * L * L / 4);
// triangular (linear) load, fixed-fixed: MA = qL^2/30, MB = qL^2/20 (hogging)
r = So.analyze(beam(L, [{ type: 'dist', member: 1, t1: 0, t2: 1, q1: 0, q2: q, dir: 'Y' }], 'fixed'));
chk('triangle FEM A', r.at(1, 0, 1).M, -q * L * L / 30); chk('triangle FEM B', r.at(1, L, -1).M, -q * L * L / 20);
// parabolic load fixed-fixed (0-q-0): M_end = -qL^2/15
r = So.analyze(beam(L, [{ type: 'dist', member: 1, t1: 0, t2: 1, q1: 0, qm: q, q2: 0, shape: 'parabola', dir: 'Y' }], 'fixed'));
chk('parabola FEM', r.at(1, 0, 1).M, -q * L * L / 15);
// partial parabola reversed range + equilibrium
r = So.analyze(beam(L, [{ type: 'dist', member: 1, t1: 0.8, t2: 0.2, q1: 1, qm: 5, q2: 2, shape: 'parabola', dir: 'Y' }]));
chk('partial total', r.reactions[0].Ry + r.reactions[1].Ry, 3.6 * (1 + 4 * 5 + 2) / 6);
chk('sys size', r.sys.K.length, r.kin);
for (const [e, x, v] of [['2x^2+1', 2, 9], ['-x^2', 3, -9], ['2(x+1)(x-1)', 3, 16], ['sqrt(abs(-16))+e^0', 0, 5], ['1.5e1*x', 2, 30], ['L/2-x', 1, 2]]) chk('expr ' + e, So.compileExpr(e)(x, 6), v);
try { So.compileExpr('alert(1)'); console.log('FAIL bad expr accepted'); } catch (err) { console.log('PASS bad expr rejected:', err.message); }
