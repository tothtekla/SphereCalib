# Oriented sphere solver: mathematical audit

Status: initial derivation audit, 2026-11-18. This is not a novelty proof and not a publication claim.

## Problem and assumptions

Let the calibrated camera center be the origin, the known sphere radius be `r > 0`, and the unknown sphere center be `s in R^3`, with `||s|| > r`. A detected silhouette pixel defines a unit viewing ray `u`. Its image tangent defines a plane through the camera center; let `n` be either unit normal of that plane, so `n.u = 0`. The sign of `n` is not observable from an unoriented image tangent.

An ordinary contour point supplies the scalar tangency constraint

`(u.s)^2 = ||s||^2 - r^2`,

together with forward visibility `u.s > 0`.

For an oriented contour measurement, the 3D tangent point is on the ray, `q = lambda*u`, and the tangent-plane contact relation is

`s - q = sigma*r*n`, where `sigma in {-1,+1}`.

Therefore

`s = lambda*u + sigma*r*n`.

One oriented observation restricts the center to one of two parallel lines and gives two independent constraints on the three center coordinates. It is not a complete sphere-center solution.

## Existing 1.5p formulation

The repository's “1.5p” sample comprises one oriented observation `(u,n)` and a second ordinary contour ray `v`. It uses two constraints from the oriented observation and one scalar tangency constraint from `v`.

Substitute `s = lambda*u + sigma*r*n` into the second-ray constraint. With `c = u.v` and `d = n.v`:

`(lambda*c + sigma*r*d)^2 = lambda^2`.

The two algebraic branches are

`lambda_+ = sigma*r*d / (1-c)`

and

`lambda_- = -sigma*r*d / (1+c)`.

On the first branch, `v.s = lambda_+`; choosing the normal sign so `lambda_+ > 0` permits both contact depths to be positive. On the second branch, `v.s = -lambda_-`, so the two contact depths have opposite signs unless `lambda = 0`. The second branch is therefore incompatible with two forward contour rays in the stated camera model.

The current implementation in `fit_sphere_oriented` is algebraically the first branch because

`1-u.v = 0.5*||u-v||^2`.

### Conditioning

The denominator is

`1-u.v = 2*sin^2(theta/2)`,

where `theta` is the angle between the two rays. Hence the estimator becomes rapidly ill-conditioned as the samples approach one another. Close samples are not merely a software corner case: short visible arcs and small projected spheres make them likely.

The present threshold `1e-12` prevents division by an almost-zero number but is not a scientifically justified conditioning policy. Evaluation must report error versus ray separation, Jacobian singular values, and rejection rate.

### What is verified now

- The implementation reproduces exact synthetic centers for several on-axis/off-axis scenes, radii, intrinsics, tangent-sign flips, and reversed sample order.
- Coincident rays and invalid normals are rejected.
- The root/visibility argument above is consistent with the implemented branch.

### What remains unverified

- independent symbolic simplification and automatic differentiation checks;
- finite-difference sensitivity and condition-number maps;
- numerical behavior close to degeneracy;
- correlated point/tangent measurement noise;
- effects of intrinsic, distortion, and radius errors;
- runtime numerical parity fixtures against the original MATLAB Direct3pFit code;
- novelty relative to the complete oriented-feature and silhouette literature.

## Two-oriented-point (2OP) formulation

For two oriented observations `(u1,n1)` and `(u2,n2)`, each generates candidate center lines

`L_i(sigma_i): s = a_i + lambda_i*u_i`, with `a_i = sigma_i*r*n_i`.

For each of four sign pairs `(sigma1,sigma2)`, solve

`[u1, -u2] [lambda1, lambda2]^T = a2-a1`.

With exact compatible observations, the correct pair has zero line-intersection residual and the two line points coincide at the sphere center. With noisy observations the lines are generally skew; a simple candidate is the midpoint of their closest points, but that is only an unweighted geometric heuristic, not a maximum-likelihood estimator.

### Degrees of freedom and uniqueness

- Sphere center with known radius: three unknown degrees of freedom.
- One ordinary contour point: one scalar constraint.
- One oriented contour point: two independent constraints; the center remains on a line, with a discrete normal-sign ambiguity.
- 1.5p: nominally three independent constraints and a unique forward solution outside degeneracies.
- 2OP: four scalar constraints for three unknowns; it is overconstrained in the center parameters even though it uses only two image locations.

Calling 2OP a two-point minimal solver is potentially misleading. Each tangent requires a spatial neighborhood or fitted contour, and the second oriented observation contributes redundant consistency information in the noise-free center model. The correct claim, if supported, may be improved hypothesis verification or conditioning, not a lower algebraic degree of freedom.

### 2OP degeneracies and ambiguities

- `u1` parallel or nearly parallel to `u2`: the two candidate lines cannot be stably intersected.
- close contour samples: both line directions and tangent estimates are strongly correlated.
- normal-sign ambiguity: four sign pairs require forward-depth and residual selection; symmetric/near-degenerate cases may leave multiple plausible candidates.
- noisy skew lines: midpoint fitting lacks a covariance model and can be biased.
- tangent bias: both candidate lines can agree closely yet intersect at a wrong center if their tangents share systematic bias.
- radius error: line offsets scale directly with the assumed radius.

## Orientation-aware refinement audit

Current refinement stacks point angular residuals and orientation residuals, divides them by fixed independent scales, and applies a `soft_l1` loss. This is useful as an ablation but not yet a statistically valid joint likelihood because position and tangent are computed from the same local pixels.

Risks include:

- double counting the same edge evidence;
- ignoring point/tangent covariance and neighboring-sample correlation;
- arbitrary relative weighting;
- non-smooth behavior from unsigned normal handling;
- confidence values based only on gradient magnitude rather than calibrated uncertainty;
- bias from blur, illumination, texture, highlights, and occlusion boundaries.

A defensible refinement comparison needs at least: point-only refinement; heuristic orientation refinement; covariance-aware refinement; oracle analytic tangents; independently perturbed tangents; and image-estimated tangents. All must start from paired hypotheses or the same center initialization where the refinement itself is being isolated.

## Independent verification plan

1. Implement a reference 2OP solver separately from the production path and test all four sign pairs.
2. Use symbolic algebra to verify the 1.5p branches and 2OP line equations.
3. Add property-based exact tests across randomized centers, radii, intrinsics, tangent signs, and point order.
4. Add finite-difference Jacobian tests for every residual and solver output away from branch changes.
5. Record singular values, line angle, ray separation, sign choice, and closest-line residual for every hypothesis.
6. Generate adversarial sweeps near each degeneracy, including failures rather than clipping them away.
7. Compare simple 2OP midpoint, covariance-weighted 2OP, 1.5p, 3p, and full conic estimation using identical observations.

## Current mathematical conclusion

The 1.5p formula has a coherent geometric derivation and exact synthetic checks, but its numerical stability and statistical value are not established. The 2OP construction follows directly from intersecting two center-candidate lines defined by oriented silhouette observations. On present evidence, that construction should be treated as an obvious geometric baseline whose novelty is unproven. The potentially stronger contribution is a rigorously evaluated, reliability-aware robust estimator and refinement model showing exactly when tangent information helps or harms.
