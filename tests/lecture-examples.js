const So=require('../solver.js');
const N=(id,x,y,sup,o)=>Object.assign({id,x,y,sup:sup||'none',ang:0},o||{});
const M=(id,n1,n2,o)=>Object.assign({id,n1,n2,E:1,A:1,I:1,rel:[0,0,0,0,0,0]},o||{});
const f=v=>Math.abs(v)<1e-9?0:+v.toFixed(5);
function rep(name,r){ if(!r.ok){console.log(name,'FAIL',r.errors);return;} console.log(name,'n=',r.det.n, r.reactions.map(R=>`${R.node}:(${f(R.Rx)},${f(R.Ry)},${f(R.M)})`).join(' '), r.warnings.join('|')); }
// 1 Gerber beam, lecture 1: expect Ay=2.5 By=6.5 Cy=4.5 Dy=1.5 ; M: 7.5,-3,6,-2,3
let g={settings:{},nodes:[N(1,0,0,'pin'),N(2,6,0,'roller'),N(3,7,0),N(4,12,0),N(5,13,0,'roller'),N(6,17,0,'roller')],
 members:[M(1,1,2),M(2,2,3,{rel:[0,0,0,0,0,1]}),M(3,3,4,{rel:[0,0,0,0,0,1]}),M(4,4,5),M(5,5,6)],
 loads:[{type:'point',member:1,t:0.5,Py:6},{type:'point',member:3,t:0.4,Py:5},{type:'point',member:5,t:0.5,Py:4}]};
let r=So.analyze(g); rep('gerber',r);
console.log(' M@3',f(r.at(1,3,-1).M),'M@B',f(r.at(1,6,-1).M),'M@9',f(r.at(3,2,-1).M),'M@C',f(r.at(4,1,-1).M),'M@15',f(r.at(5,2,-1).M),'S',f(r.at(1,1).S),f(r.at(1,4).S),f(r.at(2,.5).S),f(r.at(3,3).S),f(r.at(5,1).S),f(r.at(5,3).S));
// 2 portal frame lecture 3: a=1,q=1: HA=-3, VB=2.25, dB=-189/8 (unit →?), thC=1.5
let p={settings:{axialRigid:true},nodes:[N(1,0,0,'pin'),N(2,0,3),N(3,2,3),N(4,2,0,'roller')],
 members:[M(1,1,2),M(2,2,3,{I:2}),M(3,3,4)],loads:[{type:'dist',member:1,t1:0,t2:1,q1:1,q2:1,dir:'X'}]};
r=So.analyze(p); rep('portal',r); console.log(' uB',f(r.disp.get(4).u),'(expect 23.625) thC',f(r.disp.get(2).r),'(±1.5)', 'M@C',f(r.at(1,3).M),f(r.at(2,0).M));
let mm=So.unitLoad(p,{node:4},'x'); console.log(' MM total',f(mm.total),'direct',f(mm.direct),mm.rows.map(x=>f(x.bend)));
// 3 propped cantilever q: RB=3qL/8=3.75 (L=10,q=1), MA=qL²/8=12.5
let pc={settings:{},nodes:[N(1,0,0,'fixed'),N(2,10,0,'roller')],members:[M(1,1,2)],loads:[{type:'dist',member:1,t1:0,t2:1,q1:1,q2:1,dir:'Y'}]};
r=So.analyze(pc); rep('propped',r);
// 4 simply supported q: mid deflection 5qL^4/384 = 130.208 (L=10), with Maxwell-Mohr at midpoint
let ss={settings:{},nodes:[N(1,0,0,'pin'),N(2,10,0,'roller')],members:[M(1,1,2,{A:1e6})],loads:[{type:'dist',member:1,t1:0,t2:1,q1:1,q2:1,dir:'Y'}]};
r=So.analyze(ss); rep('ss',r); console.log(' vmid',f(r.at(1,5).v),'Mmid',f(r.at(1,5).M));
mm=So.unitLoad(ss,{member:1,t:0.5},'y'); console.log(' MM',f(mm.total),f(mm.direct));
// 5 cantilever tip load + rot spring k: delta = Pl^3/3EI + Pl^2/k ; l=2,P=3,k=4 -> 8+3=11
let cs={settings:{},nodes:[N(1,0,0,'pin',{kr:4}),N(2,2,0)],members:[M(1,1,2)],loads:[{type:'point',node:2,Py:3}]};
r=So.analyze(cs); rep('spring',r); console.log(' v',f(r.disp.get(2).v)); mm=So.unitLoad(cs,{node:2},'y'); console.log(' MM',f(mm.total),f(mm.direct),f(mm.tot.spring));
// 6 lecture 2 slider frame: W at D, expect relative ΔuB = -W l²/(6EI) ; build: A(0,0) pin, D(0,1), B(1,1), C(1,2) pin; col BC connected via horizontal slider at B => member BC end at B has release of local shear? BC vertical: horizontal = local shear (S). 
let sl={settings:{axialRigid:true},nodes:[N(1,0,0,'pin'),N(2,0,1),N(3,1,1),N(4,1,2,'pin')],members:[M(1,1,2),M(2,2,3),M(3,3,4,{rel:[0,1,0,0,0,0]})],loads:[{type:'point',node:2,M:-1}]};
r=So.analyze(sl); rep('slider',r); { const a=r.at(3,0,1), b=r.disp.get(3); console.log(' u beam side',f(b.u),'u col side',f(a.ux),'rel',f(b.u-a.ux),'(expect ±1/6=0.16667)'); }
// 7 temperature cantilever gradient: tip v = kappa L^2/2 ; alpha=1e-5, Tb-Tt=20,h=0.2,L=3 => kappa=1e-3 → 0.0045 up(bottom hotter → curves up? bottom elongates → concave up → tip up)
let tc={settings:{},nodes:[N(1,0,0,'fixed'),N(2,3,0)],members:[M(1,1,2,{alpha:1e-5,Tt:0,Tb:20,h:0.2})],loads:[]};
r=So.analyze(tc); rep('temp',r); console.log(' v tip',f(r.disp.get(2).v),'u',f(r.disp.get(2).u)); mm=So.unitLoad(tc,{node:2},'y'); console.log(' MM',f(mm.total),f(mm.direct));
// 8 settlement lecture4: simply supported, D sinks Δs → mid = Δs/2
let se={settings:{},nodes:[N(1,0,0,'pin'),N(2,4,0,'roller',{dy:-0.02})],members:[M(1,1,2)],loads:[]};
r=So.analyze(se); rep('settle',r); console.log(' vmid',f(r.at(1,2).v)); mm=So.unitLoad(se,{member:1,t:.5},'y'); console.log(' MM',f(mm.total),f(mm.direct));
// 9 truss: triangle, P down at apex
let tr={settings:{},nodes:[N(1,0,0,'pin'),N(2,4,0,'roller'),N(3,2,2)],members:[M(1,1,2,{rel:[0,0,1,0,0,1]}),M(2,1,3,{rel:[0,0,1,0,0,1]}),M(3,2,3,{rel:[0,0,1,0,0,1]})],loads:[{type:'point',node:3,Py:10}]};
r=So.analyze(tr); rep('truss',r); console.log(' N',[1,2,3].map(i=>f(r.at(i,1).N)),'(5, -7.07107, -7.07107)');
// 10 inclined roller 45°, mechanism test
let ir={settings:{},nodes:[N(1,0,0,'pin'),N(2,4,0,'roller',{ang:45})],members:[M(1,1,2)],loads:[{type:'point',member:1,t:.5,Py:10}]};
r=So.analyze(ir); rep('incl',r);
let me={settings:{},nodes:[N(1,0,0,'roller'),N(2,4,0,'roller')],members:[M(1,1,2)],loads:[{type:'point',member:1,t:.5,Py:10}]};
r=So.analyze(me); rep('mech',r);
// 11 influence line simply supported M at mid: peak L/4=2.5
let il=So.influence(ss,{kind:'internal',member:1,t:.5,what:'M'},10); console.log(' IL M',il[0].pts.map(p=>f(p.val)).join(','));
il=So.influence(ss,{kind:'internal',member:1,t:.3,what:'S'},10); console.log(' IL S',il[0].pts.map(p=>f(p.val)).join(','));
il=So.influence(ss,{kind:'reaction',node:1,comp:'y'},5); console.log(' IL RA',il[0].pts.map(p=>f(p.val)).join(','));
// 12 trapezoid partial + equilibrium
let tp={settings:{},nodes:[N(1,0,0,'fixed'),N(2,3,4)],members:[M(1,1,2)],loads:[{type:'dist',member:1,t1:.2,t2:.8,q1:2,q2:5,dir:'proj'},{type:'dist',member:1,t1:0,t2:1,q1:1,q2:1,dir:'perp'}]};
r=So.analyze(tp); rep('trap',r); console.log(' eq',r.equilibrium);
