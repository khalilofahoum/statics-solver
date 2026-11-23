# Statics Solver

A browser-based helper for a structural analysis (statics of structures) course: draw a plane beam, frame or truss, load it, and get the diagrams and the hand-method calculations.

It runs entirely in the browser with no server and no build step. Open `index.html`.

## What it does

**Model**

- Members on a snap grid, joints, internal hinges, shear and axial sliders
- Supports: pin, roller (any inclination), clamp, sliding clamp
- Springs: vertical, horizontal and rotational
- Loads: point forces and moments, linear (uniform / trapezoid / triangle), parabolic, and any function `q(x)`
- Temperature (uniform and gradient), lack of fit, support settlements

**Results**

- Reactions, N, S and M diagrams (M drawn on the tension side), deformed shape
- Values of N, S, M and the displacements at any point of any member
- Influence lines for M, S, N at a section or for a support reaction

**Calculations tab** (formulas typeset with KaTeX)

1. Static indeterminacy
2. Reactions and equilibrium check
3. N(x), S(x), M(x) for every member
4. Joint displacements
5. Unit-load method (Maxwell–Mohr)
6. Force (flexibility) method with redundants you choose
7. Stiffness (displacement) method
8. Slope-deflection
9. Moment distribution (Cross), including sway
10. Influence lines

## Sign convention

- N > 0 is tension; S = dM/dx and q = −dS/dx
- M > 0 puts the dashed fibre of a member in tension
- Loads: Px → positive, Py ↓ positive, M ↺ positive, q ↓ positive
- Reactions and displacements: → and ↑ positive

## Running it

- **Directly:** download the repository and open `index.html` in a browser.
- **Windows installer:** extract `setup/StaticsSolver-setup.zip` and run `install.bat`. It copies the program to `%LOCALAPPDATA%\StaticsSolver`, downloads KaTeX and creates a Desktop icon.

## Files

| File | Purpose |
|---|---|
| `index.html`, `style.css`, `app.js` | Editor, drawing and the calculation report |
| `solver.js` | Direct stiffness solver, unit-load integrals, force method, influence lines |
| `classic.js` | Slope-deflection and moment distribution |
| `vendor/katex` | KaTeX 0.16.9 (MIT licence), used to typeset the formulas |
| `setup/` | Windows installer |
| `tests/` | Checks of the solver against textbook results |

## Tests

```
node tests/lecture-examples.js
node tests/force-method.js
node tests/load-types.js
node tests/slope-deflection-cross.js
```

## Limits

- Linear elastic, small displacements, plane structures; shear deformation is neglected.
- Slope-deflection and moment distribution cover beams and frames under loads only (no springs, settlements, temperature or sliders).
- Plastic analysis is not included.
- This is a study aid. Check results independently before relying on them.
