# Animation error budget: per-bone precision and end effector error

## Short answer

No. A per-bone precision threshold does not control the geometric error of an end
effector in the general case. It controls the error that ACL measures on a rigid
shell around each bone. That shell has a radius that you supply: `shell_distance`.

A per-bone threshold becomes a bound on the end effector error only when all of
these conditions hold:

1. For each bone that influences the vertices of interest, the shell radius is at
   least the distance from the bone origin to those vertices.
2. The runtime transform type and the transform metric match the runtime.
3. The bit rate search attains the target. ACL does not report a failure.
4. The transform space is local, or you assign the budget to the bone that owns
   the vertices.
5. The dominance rule does not replace the precision of the bone with a larger
   value from a descendant.

The rest of this report gives the source evidence for each condition, the
relation between local track error and object space error, the information that
the bit rate allocation uses, a configuration guide, the split of
responsibility, and an acceptance standard.

## 1. What a per-bone precision threshold measures

`track_desc_transformf` holds two values for each transform track:
`precision` and `shell_distance` (`includes/acl/core/track_desc.h:116-128`).
The header states the intent:

- `precision`: the target at the shell.
- `shell_distance`: the radius of the shell, in the units of the clip.

Compression converts the target into the units of the clip and keeps the radius:

```c++
// includes/acl/compression/impl/clip_context.impl.h:179-184
metadata.parent_index = get_error_parent_index(space, desc.parent_index);
// The error at playback is the error in the units of the clip multiplied by the playback scale
metadata.precision = desc.precision / track_list.get_playback_scale();
metadata.shell_distance = desc.shell_distance;
```

The same division occurs in the clip adapter
(`includes/acl/compression/impl/transform_clip_adapters.h:233-238`). Thus the
precision is in playback units, and the shell radius is in clip units
(`includes/acl/compression/track_array.h:143-153`).

The metric places points on a sphere of radius `r` in the local space of the
transform and transforms them with the raw and the lossy transform
(`includes/acl/compression/transform_metrics.h:88-131`). With a scale, it uses
three points, one on each axis. Without a scale, it uses two points
(`includes/acl/compression/transform_metrics.h:242-265`). The result is the
largest distance between the transformed points.

Define these symbols for one bone at one sample time:

- `T0`: the raw transform. `T1`: the lossy transform.
- `r`: `shell_distance`.
- `e(r)`: the measured error at the shell. This is the value that compression
  compares to `precision`.

### The lever bound

For a rigid transform pair, the worst point error grows with the radius of the
shell. The code computes the exact value with double precision
(`includes/acl/compression/transform_metrics.h:151-191`):

- The translation delta is a constant with respect to the radius.
- The rotation delta contributes `2 * R * sin(theta / 2)`, where `theta` is the
  rotation delta.
- The two contributions combine under the norm
  (`includes/acl/compression/transform_metrics.h:182-190`).

The same form applies to a VQ transform
(`includes/acl/compression/transform_metrics.h:226-236`). For a VQS transform,
the code adds the scale term `|s1 - s0| * R`
(`includes/acl/compression/transform_metrics.h:193-224`).

Let `e_precise(R)` be the value of this function. It is a norm of a constant term
and a term that is linear in `R`, and the radius inside the function is scaled
linearly in every branch. Thus `e_precise(R) / R` does not increase with `R`.
Therefore, for `R >= r`:

```
e_precise(R) <= (R / r) * e_precise(r)                        (1)
```

and for `R < r`:

```
e_precise(R) <= e_precise(r)                                  (2)
```

The same ratio argument holds for the sampled shell metric. It transforms two or
three points, and each point is a constant term plus a linear term in `R`. The
maximum of non-increasing ratios does not increase
(`includes/acl/compression/transform_metrics.h:242-265`). Thus the scaling law
holds for every built-in metric:

```
measured error at radius R <= max(1, R / r) * measured error at radius r   (3)
```

The library has the exact sphere maximum only for VQ and VQS transforms, through
`precise_transform_metric` (`includes/acl/compression/transform_metrics.h:310-352`).
For the other types the shell metric samples axis points, and its header calls
the result an approximation (`includes/acl/compression/transform_metrics.h:269-275`).
This approximation affects the value at radius `r`. It does not affect the
scaling law in equation (3). It means that `e(r)` itself can be below the true
worst point error at radius `r`. Keep a margin for a 3D scale or a shear.

Equation (3) is the bridge between the shell threshold and the geometry. Two
consequences follow.

- If `r >= R`, the shell threshold bounds the vertex error directly. This is the
  only case where a per-bone threshold is a geometric bound.
- If `r < R`, the vertex error can be larger by the factor `R / r`. The shell
  threshold does not bound the vertex error.

For a VQS transform, the metric multiplies the radius by the uniform scale
before it measures (`includes/acl/compression/transform_metrics.h:216-220`).
The distance from the bone origin to the vertices grows by the same factor.
Thus the ratio `R / r` is the same in the local space of the bone and at
playback.

### Example

A bone has `r = 3.0` cm and `precision = 0.01` cm. The vertices that it deforms
can be 25 cm from the bone origin, for example the forearm surface at the wrist.
The bound of equation (1) gives:

```
vertex error <= (25 / 3) * 0.01 = 0.083 cm
```

The end effector error can be up to 8.3 times the value that the threshold
suggests. To hold 0.01 cm at 25 cm, set `r = 25` or set
`precision = 0.01 * 3 / 25 = 0.0012` cm.

Only the ratio `precision / r` controls the bound. To make the input
self-documenting, set `r` to the reach of the geometry and keep `precision` equal
to the vertex budget. Then the shell error is the worst vertex error, up to the
metric approximations in section 3.

## 2. Local track error and object space error

The two quantities are not the same. This section shows where ACL uses each one.

### Local error is only an approximation

The v1 bit rate search first computes bit rates from the local space error, then
refines them in object space
(`includes/acl/compression/impl/quantize.transform.h:1258-1350`,
`includes/acl/compression/impl/quantize.transform.h:1733-1760`). The source
states the relation:

```
// includes/acl/compression/impl/quantize.transform.h:1665-1675
// If each parent bone has infinite precision, the local space error is equivalent.
// Since parents are lossy, it is a good approximation.
```

Thus a small local error does not prove a small object space error. A parent
rotation moves every child origin.

### ACL measures the composed object pose, so the chain lever is included

The v2 search does not measure the local error. It measures object space error:

- It converts the raw and the lossy local poses into the runtime transform type
  and composes each pose into object space
  (`includes/acl/compression/impl/quantize.transform.h:2200-2213`).
- For each transform that it optimizes, it builds a critical set that contains
  the transform itself and all of its descendants
  (`includes/acl/compression/impl/quantize.transform.h:2240-2249`).
- For each critical transform, it composes
  `measured_to_transform * transform_being_optimized * transform_to_root` and
  compares the result to the raw object transform
  (`includes/acl/compression/impl/bit_rate_policies.transform.h:381-413`,
  `includes/acl/compression/impl/quantize.transform.h:1060-1133`).

The cached transform of a non-optimized transform holds its object space
transform. The cached transform of a critical transform holds the delta from the
last optimized ancestor
(`includes/acl/compression/impl/bit_rate_policies.transform.h:79-86`).
Therefore the measured error at a descendant contains the accumulated loss of
all of its ancestors.

This gives an important result: the lever from a parent rotation to a descendant
**origin** is measured at the descendant. You do not need to sum the chain error
by hand. The residual lever is inside one bone: from the bone origin to the
vertices that it deforms. Equation (3) covers that part.

The same is true for the final measurement function. `calculate_compression_error`
composes the complete lossy local pose into object space
(`includes/acl/compression/impl/track_error.impl.h:395-396`), then measures each
bone with its own shell radius
(`includes/acl/compression/impl/track_error.impl.h:398-407`). It returns the
largest squared error, converted to playback units:

```c++
// includes/acl/compression/impl/track_error.impl.h:419-420
// We computed the squared error in the units of the clip, we return it in playback units
result.error = rtm::scalar_sqrt(result.error) * args.playback_scale;
```

### What the check does not contain

These items are outside the measured value:

- The distance from a bone origin to its skin. Only `shell_distance` represents
  it. ACL does not read a mesh.
- The skinning weights. The check treats each bone as a rigid body.
- The algebra of the engine, when it differs from the selected runtime transform
  type. ACL composes the raw pose with the selected adapter too. Thus a
  composition difference, for example a dropped shear, is not part of the error.
  The documentation states the risk
  (`docs/error_metrics.md:10`).
- The mesh and mesh rotation spaces. In mesh space a transform has no parent for
  the error (`includes/acl/compression/impl/transform_space.impl.h:60-66`,
  `includes/acl/compression/transform_space.h:51-67`). A parent rotation then
  does not move its children in the measurement. The end effector error depends
  on the budget of the bone that owns the vertices.
- The sample times between retained keys. The measurement function uses nearest
  rounding (`includes/acl/compression/impl/track_error.impl.h:716-717`). It
  checks retained samples only.

## 3. Non-uniform scale and different shell distances

### Non-uniform scale

The shell points are transformed by the transform, so a non-uniform scale makes
the effective shell an ellipsoid. The metric samples three axis points and
returns the largest error. The mid point between the axes is computed in the
arguments (`includes/acl/compression/transform_metrics.h:118-130`), but the
error function drops the W lane when a scale is present
(`includes/acl/compression/transform_metrics.h:247-265`). The header states that
the result is an approximation for a 3D scale or a shear
(`includes/acl/compression/transform_metrics.h:269-275`).

For `vqf` and `vqsf`, the precise metric computes the exact worst point
(`includes/acl/compression/transform_metrics.h:310-352`). It does not support a
3D scale or a shear. For `vqvf`, `vqmf`, `matrix3x4f`, and `custom`, the shell
metric is the only built-in option.

A VQV product discards the shear
(`includes/acl/compression/transform_adapters.h:352-374`). The raw pose is
composed with the same VQV math, so the composition error of the raw pose is not
measured. This is correct only when the engine also uses VQV.

### Different shell distances on different bones

Every bone has its own `r` and its own `p`. The bound is therefore
inhomogeneous. For a vertex with skinning weights `w_b` that sum to 1, the
skinned position error is at most the weighted sum of the per-bone point errors:

```
vertex error <= sum_b w_b * max(1, R_b / r_b) * e_b(r_b)      (4)
```

If each bone attains its target, this becomes:

```
vertex error <= sum_b w_b * max(1, R_b / r_b) * p_b           (5)
```

If all bones use the same budget `B`, the shell radius is at least the bone
reach, and the weights sum to 1, equation (5) gives `vertex error <= B`. This is
a simple and safe policy: use one budget, and set `r_b` to the reach of the
geometry that bone `b` deforms.

With one global `3 cm` shell, equation (5) fails for every bone whose deformed
geometry is longer than 3 cm. This is the usual cause of a visible error while
the local targets pass.

### The dominance rule can replace a precision

The library computes a dominant descendant for each bone. It walks from the
leaves to the roots. A child becomes dominant when the child shell distance plus
the distance to the parent is larger than the parent shell distance
(`includes/acl/compression/impl/rigid_shell_utils.h:149-175`). When this
happens, the parent adopts the shell distance **and the precision** of the child:

```c++
// includes/acl/compression/impl/rigid_shell_utils.h:169-174
if (new_parent_shell_distance > parent_shell.local_shell_distance)
{
    parent_shell.local_shell_distance = new_parent_shell_distance;
    parent_shell.precision = transform_shell.precision;
    parent_shell.dominant_transform_index = transform_shell.dominant_transform_index;
}
```

The v2 search uses the propagated precision for its acceptance test
(`includes/acl/compression/impl/quantize.transform.h:2443-2444`). The radius
comes from the authored metadata
(`includes/acl/compression/impl/quantize.transform.h:1072`). Therefore a parent
can be checked against the precision of a descendant. If that precision is
larger, the parent is allowed more error than its own description requests.
With one uniform precision this cannot occur. With mixed precisions, it can.

The v1 search uses both propagated values. Its dominance path takes the radius
and the precision from the propagated metadata
(`includes/acl/compression/impl/quantize.transform.h:818-833`). The same
substitution can therefore occur there, and it also changes the radius.

Constant folding uses the whole-clip dominance result for both values
(`includes/acl/compression/impl/compact.transform.h:97-131`). A wrong shell
distance can therefore fold a sub-track to a constant while the surface error is
larger than expected.

## 4. Information that the bit rate allocation uses

The allocation does not use a byte budget. It searches bit rates per transform
and accepts the first size that meets the error targets. The search uses these
inputs.

| Information | Use | Source |
| --- | --- | --- |
| Per-bone precision, divided by the playback scale | Error target for each critical transform | `clip_context.impl.h:183`, `quantize.transform.h:2443-2444` |
| Per-bone shell distance | Radius of the error measurement | `clip_context.impl.h:184`, `quantize.transform.h:1072` |
| Object space scale of each sample | Converts the shell to object space for dominance | `rigid_shell_utils.h:128-143` |
| Distance from each bone origin to its parent origin, largest over samples | Dominance test, v1 chain error | `rigid_shell_utils.h:134-145`, `rigid_shell_utils.h:166-174` |
| Hierarchy: parent, children, descendants, leaves, depth | Critical sets, evaluation order, chain rebuild | `topology_metadata.h:50-109`, `quantize.transform.h:2240-2249` |
| Raw and lossy sample poses composed in object space | The measured error | `quantize.transform.h:2200-2213`, `quantize.transform.h:1060-1133` |
| Runtime transform type and transform metric | Composition and measurement | `transform_runtime.h:129-166` |
| Bit rate permutation tables, one per number of degrees of freedom, sorted by size | Search order | `quantize.transform.h:2224-2230`, `bit_rate_permutations.h` |
| First size that meets all critical targets | Stops the search | `quantize.transform.h:2406-2409`, `quantize.transform.h:2435-2444` |
| Compression level | Wider permutation search in v1 only | `quantize.transform.h:1772`, `quantize.transform.h:1804` |
| Clip length and longest chain length | Automatic level selection | `compress.transform.impl.h:127-149` |
| Keyframe stripping threshold, proportion, trivial flag | Removes whole frames | `quantize.transform.h:2845-2853`, `keyframe_stripping.h:69-102` |
| Per-bone wrap error at the shell | Loop detection | `optimize_looping.transform.h:180-196` |
| Playback scale | Divides the precision; multiplies the reported error | `clip_context.impl.h:183`, `track_error.impl.h:420` |

The allocation does **not** use these inputs:

- The mesh, the vertex positions, or the skinning weights.
- The distance from a bone to the associated surface. Only the authored
  `shell_distance` carries this information.
- A per-clip or per-scene error budget in bytes or in distance.
- The end effector position, the sockets, or the contact points.
- The bind pose geometry, except the translation of each bone relative to its
  parent.

## 5. Configuration guide for size and end precision

### Step 1: derive the shell radius from the geometry

For each bone `b`, compute `R_b`: the largest distance from the bone origin to a
vertex that bone `b` deforms, in the bind pose, in clip units. Use the vertices
with a non-zero weight for `b`. Set `shell_distance = R_b`.

ACL cannot compute this value. The compressor tool reads it from the clip file as
`vertex_distance` (`includes/acl/io/clip_reader.h:637-675`). The value is an
input of the caller.

For a limb, `R_b` is close to the length of the deformed segment, not to the
radius of the limb. A forearm bone can deform vertices 25 cm from the elbow. A
3 cm shell then underestimates the surface error by a factor of about 8.

### Step 2: set the budget

Pick one vertex budget `B` for the character, in playback units. Set
`precision = B` on every bone, and set `shell_distance = R_b`. Equation (5) then
keeps every skinned vertex within `B`, if the target is attained.

If a small shell is necessary for memory, use the ratio:

```
precision_b = B * r_b / R_b                                 (6)
```

The ratio `precision / r` is the quantity that the search actually controls.
Keep this ratio consistent across the bones that share a vertex. A smaller
shell also saves memory, because the search then accepts a larger loss for the
same measured value.

### Step 3: select the runtime transform type

Select the type that the runtime uses. The documentation states the rule: when
the measured error is not the error of the runtime, the visual quality can be
lower (`docs/error_metrics.md:10`). The selection costs differ.

| Runtime type | Composition error | Metric | Relative size | Use when |
| --- | --- | --- | --- | --- |
| `vqf` | None, no scale | Precise, exact | Smallest | The clip has no scale |
| `vqsf` | None, uniform scale only | Precise, exact | One scale value per sample | The raw tracks are `vqsf` and the base scale is uniform |
| `vqmf` | None | Shell, approximate | Larger | Default with scale, engine keeps shear |
| `vqvf` | Drops shear | Shell, approximate | Larger than VQ | The engine composes with VQV |
| `matrix3x4f` | None | Shell, approximate | Largest | The engine uses matrices |
| `custom` | Your algebra | Your metric | Depends | The engine uses another type |

The selector uses `vqf` for a clip with no scale when the metric supports it
(`includes/acl/compression/impl/transform_runtime.h:141-146`). With `automatic`,
a `vqsf` clip uses `vqsf`, and the other clips use `vqmf`
(`includes/acl/compression/impl/transform_runtime.h:148-155`).

The runtime transform type always uses the v2 search, also with a custom adapter
or a custom metric. The adapter and the metric are called through their virtual
functions in that case
(`includes/acl/compression/impl/quantize.transform.h:2597-2602`). Only a legacy
error metric that returns false from `supports_bit_rate_algorithm_v2` uses the
v1 search (`includes/acl/compression/impl/quantize.transform.h:2721-2724`).

For a shell-metric type, keep a margin. The metric samples three axis points and
can miss the worst direction under a shear or a 3D scale
(`includes/acl/compression/transform_metrics.h:269-275`).

### Step 4: keep the padding that costs nothing

- Use `get_default_compression_settings()`. It selects the automatic level,
  variable formats, loop optimization, and trivial keyframe stripping
  (`includes/acl/compression/impl/compression_settings.impl.h:184-195`).
- Keep `keyframe_stripping.strip_trivial` if the budget is the per-bone precision.
  A trivial frame has an error at or below the precision for every bone
  (`includes/acl/compression/impl/quantize.transform.h:2845-2853`). A positive
  `threshold` must be at or below the budget.
- Do not combine keyframe stripping with database support. The settings reject
  the combination (`includes/acl/compression/impl/compression_settings.impl.h:159-160`).
- Set the playback scale to the largest scale at which the character plays
  (`docs/creating_a_raw_track_list.md:121`). A larger scale divides the precision
  and increases the size.
- Leave `metadata` at its defaults. It does not change the precision.
- Do not raise `level` to improve the error of a modern clip. The level is read
  only in the v1 search (`includes/acl/compression/impl/quantize.transform.h:1772`,
  `includes/acl/compression/impl/quantize.transform.h:1804`). A runtime transform
  type uses v2, which ignores the level. `find_best_compression_level` still sets
  it, but nothing reads it on that path
  (`includes/acl/compression/impl/compress.transform.impl.h:127-149`).

### Worked example

A three-bone chain. The budget is `B = 0.02` cm. The bind pose gives the reach of
each bone.

| Bone | Reach `R_b` (cm) | Shell `r_b` (cm) | Precision (cm) | Bound at the surface (cm) |
| --- | --- | --- | --- | --- |
| Upper arm | 30 | 30 | 0.02 | 0.02 |
| Hand | 4 | 4 | 0.02 | 0.02 |
| Finger tip | 1.5 | 1.5 | 0.02 | 0.02 |

The same budget with the default `3 cm` shell gives these bounds from equation
(1): 0.2 cm at the upper arm, 0.027 cm at the hand, and 0.02 cm at the finger.
The arm is the limiting bone, and the end effector error can be 10 times the
target.

## 6. Input premises: what the code checks, and what you must supply

### Checked by the existing implementation

| Premise | Check | Source |
| --- | --- | --- |
| `precision` is not negative and is finite | Error result | `includes/acl/core/impl/track_desc.impl.h:52-56` |
| `shell_distance` is not negative and is finite | Error result | `includes/acl/core/impl/track_desc.impl.h:58-60` |
| `parent_index` is `k_invalid_track_index` or less than the number of tracks | Error result | `includes/acl/compression/impl/track_array.impl.h:169-174` |
| The playback scale is positive and finite | Error result | `includes/acl/compression/impl/track_array.impl.h:145-147` |
| All samples are finite | Error result | `includes/acl/compression/impl/compress.transform.impl.h:260-266` |
| The transform space is a valid value | Error result | `includes/acl/compression/impl/track_array.impl.h:142-143` |
| The runtime transform type and the metric are compatible | Error result | `includes/acl/compression/impl/compression_settings.impl.h:162-174`, `includes/acl/compression/impl/transform_runtime.h:99-166` |
| A `vqsf` runtime type needs `vqsf` raw tracks and a uniform base scale | Error result | `includes/acl/compression/impl/transform_runtime.h:148-155` |
| A custom runtime type needs a custom adapter | Error result | `includes/acl/compression/impl/transform_runtime.h:107-111` |
| The base is non-empty, is in local space, and has transform tracks | Error result | `includes/acl/compression/impl/transform_base.impl.h:100-114` |
| The output indices are unique and contiguous | Error result | `includes/acl/compression/impl/track_array.impl.h:177-215` |
| At least one root exists | Assert only | `includes/acl/compression/impl/clip_context.impl.h:287` |
| The caller does not need to sort by parent | The library sorts internally | `includes/acl/compression/impl/clip_context.impl.h:57-81`, `includes/acl/compression/impl/topology_metadata.impl.h:221-238` |

### Assumptions that the code does not check

| Assumption | Consequence |
| --- | --- |
| The hierarchy is a forest. A parent cycle is invalid. | `track_array::is_valid()` checks the range only. The topology walks follow `parent_index` until `k_invalid_track_index` (`topology_metadata.impl.h:164-171`, `topology_metadata.impl.h:245-250`). A cycle has no such terminator. The walk does not stop. |
| `parent_index` matches the skeleton of the runtime. | ACL uses it as given. A wrong parent gives a wrong object space error and a wrong chain, with no error result. |
| `shell_distance` is at least the reach of the deformed geometry. | ACL uses it as given. A small value gives a small measured error (section 1). |
| The units are centimeters. | The default values assume centimeters (`includes/acl/core/track_desc.h:121-128`). ACL does not convert. |
| The runtime transform type matches the engine. | ACL measures with the selected type. A mismatch gives a measured error that is not the error of the engine (`docs/error_metrics.md:10`). |
| The playback scale is the largest runtime scale. | The precision is divided by it. A too small value over-provisions the budget. |
| The transform space is the space of the samples. | In mesh space a parent has no parent for the error (`transform_space.impl.h:60-66`). |
| Compression can attain the target. | The search keeps the best permutation when no permutation meets the target (`quantize.transform.h:2526-2534`). Compression still returns success. |

The library documentation makes two claims that need care.

- The track description says that the error is "nearly guaranteed" to be below
  the threshold (`docs/creating_a_raw_track_list.md:28`).
- The compressor guide says that the threshold and the vertex distance
  "guarantee that a certain quality is maintained", and that visible artifacts
  usually mean that the vertex distance is not conservative enough
  (`docs/compressing_raw_tracks.md:37`).

The code supports the first claim only as a target that the search tries to
reach. The published data show a worst clip at 3.98 cm with a 0.01 cm threshold,
and 99.52 percent of samples below the threshold
(`docs/paragon_performance.md:9-15`). The second claim is a recommendation to the
caller: the guarantee holds only when the shell radius covers the deformed
geometry, and the library cannot check that condition.

## 7. Who owns the error when the local target passes and the end error fails

Measure the per-bone shell error with the same settings before you assign
responsibility. Use `calculate_compression_error` with the settings and the base
that you gave to compression
(`includes/acl/compression/impl/track_error.impl.h:957-990`).

| Observation | Cause | Owner |
| --- | --- | --- |
| The per-bone shell error exceeds its precision for at least one bone | The search did not reach the target. The clip is long, the chain is deep, or the budget is tight. | ACL attainment. Record it as an ACL limit, and report the size and the target. |
| The per-bone shell error meets its precision, but a vertex error exceeds `(R / r) * p` | The metric approximation, or a composition difference between the runtime and the adapter. | ACL metric, or the caller, if the selected type is not the type of the engine. |
| The per-bone shell error meets its precision, and the vertex error is within `(R / r) * p` but above the budget | The shell radius is smaller than the reach, or the weighted sum in equation (5) exceeds the budget. | Caller. The geometry premise or the budget derivation is wrong. |
| The measured pose error matches the engine, but the engine shows a larger error | The engine applies a different scale, a different hierarchy, or a different skin. | Caller. |
| The measured pose error does not match the engine when the inputs match | The adapter does not reproduce the algebra of the engine. | Caller. The adapter is the contract. |
| A vertex error appears between retained samples | The acceptance test sampled retained keys only. | Pipeline. The acceptance test must sample real frame times. |

In short: a failure of the local target is an ACL contribution. A failure of the
geometric premise is a caller contribution. The library provides no scene level
guarantee, and it does not claim one.

## 8. Acceptance standard for pose error

Use two measurements. They answer different questions.

### Measurement A: the per-bone target

Use `calculate_compression_error` with the same settings, the same base, the
same transform space, and the same playback scale as compression. It uses the
same transform adapter and metric as the settings
(`includes/acl/compression/impl/track_error.impl.h:957-990`).

Accept when these conditions hold for every segment and every bone:

1. `max over samples of e_b(t) <= p_b`. This is the target that compression
   promises to try.
2. `max over bones of e_b(t) / p_b <= 1`. Report this ratio, not the raw maximum.
   A raw maximum mixes bones with different targets and different radii.

Record the worst bone, the worst sample time, and the worst keyframe. The
`track_error` result returns all three (`includes/acl/compression/track_error.h:53-67`).

### Measurement B: the geometric bound

Measure the object space position of probe points at each frame time. Use the
end effector, the sockets, and a sample of the skinned vertices. For each probe
vertex `v`, compute the bound from equation (5):

```
B_v = sum_b w_vb * max(1, R_vb / r_b) * p_b
```

Accept when these conditions hold:

1. `max over frames of |p_v_lossy - p_v_raw| <= B_v` for every probe.
2. `B_v <= B`, the visual budget of the project.
3. The measured value is at most the bound. If it is larger, the metric
   approximation applies, or the runtime algebra differs from the adapter.

### Sampling

The library function uses nearest rounding
(`includes/acl/compression/impl/track_error.impl.h:716-717`). It measures the
retained samples. Measurement B must use the frame times of the runtime,
including times between retained samples. Otherwise the interpolation error and
the keyframe stripping error stay hidden.

### Reporting

Report these values for each clip, and keep them with the size:

- `max_b (e_b / p_b)`, `max_v (e_v / B_v)`, and the raw maximum in playback units.
- The worst bone, time, and keyframe.
- The 99th percentile of the per-bone error over all samples, not only the
  maximum. A single bad bone can hide a good distribution, and the published
  statistics use this form (`docs/paragon_performance.md:42`).
- The compressed size, the bit rate, and the settings hash.

The regression tests use the same idea: they compare the maximum error against a
per-config threshold (`tools/acl_compressor/sources/validate_tracks.cpp:131-133`).
The shipped configs use 0.075 and do not set a per-track precision
(`test_data/configs/uniformly_sampled_quant_medium.config.sjson:12`). The
precision then comes from each track description, with a default of 0.01 cm
(`includes/acl/core/track_desc.h:123`). The difference between 0.075 and 0.01 is
direct evidence that the local threshold is a target, not a global bound. Use the
exhaustive statistics to find the worst bone and time:
`error_per_frame_and_bone` writes the error of every bone at every sample
(`includes/acl/compression/impl/write_stats.h:210-270`).

A fail on condition 1 is a size problem: raise the precision or the shell radius
for the failing chain. When the clip uses the v1 search, a higher compression
level also helps, because the level widens the permutation search in v1 only
(`includes/acl/compression/impl/quantize.transform.h:1772`,
`includes/acl/compression/impl/quantize.transform.h:1804`). A fail on condition 2
with condition 1 satisfied is a geometry problem: fix `shell_distance`, the
weights, or the budget. Do not raise the precision to mask it. That increases the
size without correcting the premise.

## 9. Implemented constraints and recommended guarantees

### Implemented, with source evidence

- A per-bone shell target. Precision and shell radius are read per transform,
  and the precision is divided by the playback scale
  (`clip_context.impl.h:179-184`).
- Object space measurement at each descendant, with the accumulated ancestor
  loss (`quantize.transform.h:1060-1133`, `quantize.transform.h:2240-2249`).
- Exact error for VQ and VQS, and an approximation for the other types
  (`transform_metrics.h:151-236`, `transform_metrics.h:269-275`).
- Dominance propagation of the shell distance and the precision
  (`rigid_shell_utils.h:149-175`).
- The playback scale divides the precision and multiplies the reported error.
  This behavior is covered by a unit test
  (`tests/sources/compression/test_transform_runtime_compression.cpp:691-754`).
- A best-effort search that keeps the best permutation on failure, with no error
  result (`quantize.transform.h:2526-2534`).
- Keyframe stripping, constant folding, and loop optimization each measure the
  error at the shell before they change the data
  (`quantize.transform.h:2845-2853`,
  `includes/acl/compression/impl/compact.transform.h:97-131`,
  `optimize_looping.transform.h:180-196`).

### Recommended, not implemented

- A check that `shell_distance` is at least the reach of the deformed geometry.
  ACL has no mesh and cannot do this.
- A hierarchy fidelity check against the runtime skeleton, and a check that the
  hierarchy is a forest. The range check is all that exists.
- A check that the compress result met the target, reported as an error result.
- An automatic budget from the skinning weights and the bind pose.
- An end effector or skinned vertex acceptance test in the library. Do this in
  the pipeline.
- A guarantee that the shell metric bounds the worst point error under a shear.
  It samples three axis points only.
- A guarantee for the mesh and mesh rotation spaces. In these spaces the parent
  does not move the child for the error.

## 10. Summary of the judgement

- A per-bone threshold controls the error at a shell. It controls the end
  effector only if the shell contains the influenced geometry.
- The chain lever is measured, because ACL checks the composed object transform
  of every descendant. You do not need to add a chain factor.
- The remaining lever is inside each bone. Set `shell_distance` to the reach of
  the geometry, and set `precision` to the vertex budget. Or keep the ratio
  `precision / shell_distance` consistent.
- A 3 cm shell is a skinning proxy. It is not a bound for a 25 cm limb.
- With mixed precisions, the dominance rule can replace a parent precision with
  a looser descendant precision. Prefer one budget per chain, or verify the
  dominated bones.
- Compression can miss the target and still return success. Measure with
  `calculate_compression_error`, and add a vertex level acceptance test.
- If the local target passes and the end effector fails, the usual owner is the
  caller: the geometry premise, the hierarchy, the runtime type, the transform
  space, or the playback scale. An ACL failure is the case where the per-bone
  shell error itself exceeds its target.
