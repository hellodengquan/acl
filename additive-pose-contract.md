# Additive pose contract

This document describes how the base pose and the transform order of an additive
clip change the decompressed skeleton pose. It derives each rule from the code.
It gives a pose equivalence criterion and the compatibility boundaries between
compression and the runtime.

## 1. The combine functions

`includes/acl/core/additive_utils.h` defines the four base formats and the
functions that combine a clip sample with its base transform.

RTM composition order: `quat_mul(lhs, rhs)` and `vqv_mul(lhs, rhs)` apply `lhs`
first, then `rhs`. The pattern is
`local_to_world = mul(local_to_parent, parent_to_world)`.

| Format | Rotation | Translation | Scale | Default scale |
|---|---|---|---|---|
| `none` | sample as is | sample as is | sample as is | 1.0 |
| `relative` | `vqv_mul(additive, base)` | part of `vqv_mul` | part of `vqv_mul` | 1.0 |
| `additive0` | `quat_mul(additive.rot, base.rot)` | `additive.trans + base.trans` | `additive.scale * base.scale` | 1.0 |
| `additive1` | `quat_mul(additive.rot, base.rot)` | `additive.trans + base.trans` | `(1 + additive.scale) * base.scale` | 0.0 |
| `custom` | the base adapter decides | the base adapter decides | the base adapter decides | 0.0 or 1.0, from the adapter |

Key points:

*  `relative` composes the full transforms (`additive_utils.h:185`). The base
   rotation and scale transform the additive translation. The additive sample
   lives in the local space of the base.
*  `additive0` and `additive1` combine each component on its own
   (`transform_add0`, `transform_add1`, `additive_utils.h:156-170`). The base
   rotation does not transform the additive translation: the code adds the two
   translations as vectors.
*  The rotation order is `quat_mul(additive, base)`: the additive rotation
   applies first, the base rotation second. Quaternions do not commute. The
   reversed order gives a different rotation unless the two rotations are
   coaxial or one of them is the identity.
*  The translation sum and the scale product commute. The rotation product does
   not. Thus `additive0` and `additive1` are sensitive to the argument order in
   the rotation only, while `relative` is sensitive in every component.
*  `get_additive_default_scale` (`additive_utils.h:150`) returns 0.0 for
   `additive1` and 1.0 for the other formats. A track without scale (for
   example a `vqf` track) gives this value to each of its samples.

The functions `convert_to_relative`, `convert_to_additive0`, and
`convert_to_additive1` (`additive_utils.h:203-222`) are the algebraic inverses.
For a unit quaternion base, `apply_additive_to_base(format, base,
convert_to_*(base, x))` returns `x`, up to float rounding. The rotation inverse
uses `quat_conjugate`, so the inverse is exact only when the base rotation has
unit length.

## 2. Where the base applies in the pipeline

### Compression

*  `transform_base_desc_t` (`includes/acl/compression/transform_base.h`) holds
   the format, the base tracks, and the optional adapter.
*  Compression combines each sample with the base first, in the transform space
   of the clip. Then it puts the result in object space and measures the error
   on the vertices of a rigid shell (`transform_base.h:44-48`,
   `impl/transform_space_context.h:241-257`).
*  The base tracks must be in local space, with the transform hierarchy of the
   clip (`transform_base.impl.h:100-116`). In mesh space, compression puts the
   base in mesh space with `rtm::vqv_mul`, parent before child: the scale of a
   parent does not shear its children. In mesh rotation space, only the
   rotations of the base go to mesh space (`transform_space_context.h:127-153`).
*  A clip sample uses the base sample at the same normalized time, rounded to
   the nearest sample. The code does not interpolate
   (`transform_space_context.h:354-365`, `sample_streams.h:560-604`). The base
   can have a different sample rate, a different number of samples, and a
   different transform type than the clip.
*  Compression does not normalize the combined transform
   (`transform_space_context.h:238-246`). The measured error is the error of a
   runtime that combines the transforms with the same function and does not
   normalize the result.

### Decompression

*  Decompression returns the additive samples as they are. It does not know the
   base and it does not combine. The runtime applies the result to the base.
*  The compressed data keeps one bit for the default scale
   (`core/impl/compressed_headers.h:112-113`). It does not keep the additive
   format. The format and the base are a contract between the tool that
   compresses and the runtime that decompresses: they travel out of band.
*  Decompression reads the default scale with
   `compressed_tracks::get_default_scale()` and writes it for the scale
   sub-tracks that are constant or missing
   (`decompression/impl/decompression.transform.h:1047`). With the
   `default_sub_track_mode::constant` mode, the track writer supplies the value
   and must agree with the format.

## 3. The four mismatch scenarios

### 3.1 The base is missing

Two forms:

*  `format` is not `none` but `tracks` is null: `transform_base_desc_t::is_valid`
   returns an error and compression stops (`transform_base.impl.h:100-101`).
   This case fails loudly.
*  `format` is `none` on a clip that is additive: compression accepts the clip
   and measures the error on the raw additive values, as if they were absolute
   local transforms. The compressed data matches the additive values within the
   precision. But the runtime combines them with a base. The base rotation
   rotates the translation error, the base scale multiplies it, and the error
   of a parent moves its children. The compression error does not bound the
   error of the final pose. This case fails silently.

### 3.2 The base is not the identity

*  The default scale depends on the format: 0.0 for `additive1`, 1.0 for the
   others. If the runtime assumes a default scale of 1.0 for an `additive1`
   clip, a constant scale sub-track decompresses to 1.0 instead of 0.0. The
   combined scale becomes `(1 + 1) * base_scale`: twice the correct value.
*  With `additive0` and `additive1`, a non-identity base rotation changes the
   final rotation but not the final translation: the translations add as
   vectors. With `relative`, the base rotation and scale also transform the
   additive translation. The same additive clip on the same non-identity base
   gives different poses under the two formats.
*  A base with a non-unit rotation breaks the inverse pair: `quat_conjugate`
   is the inverse only for unit quaternions. The conversion functions then
   return values that do not reconstruct the source transform.

### 3.3 Parent-child composition

*  Object space conversion uses `object = vqv_mul(local, parent_object)`,
   parent before child, and normalizes the result
   (`transform_space_context.h:97-109`,
   `transform_error_metrics.h:373-413`).
*  The base must have the same transform hierarchy as the clip. In mesh and
   mesh rotation space, compression asserts that the base and the clip have the
   same number of transforms (`transform_space_context.h:332`). A base with a
   different hierarchy puts each base transform on the wrong joint.
*  In local space, the combine happens per transform before the object space
   conversion: the combined error of a parent moves its children, and the error
   measurement sees it. In mesh space, the combine happens in mesh space: the
   error of a parent does not move its children. The same clip and the same
   base give a different error profile in the two spaces, and the compressed
   result can differ.
*  The mesh rotation space does not support the `relative` format
   (`transform_base.impl.h:88-90`): `relative` multiplies the whole transforms,
   but the rotations and the translations are in different spaces there.

### 3.4 The combine order differs

*  Rotation: `quat_mul(additive, base)` and `quat_mul(base, additive)` differ
   by the commutator of the two rotations. The error grows with the angle of
   both rotations. It is zero only when one rotation is the identity or the
   axes are parallel.
*  `relative`: `vqv_mul(additive, base)` and `vqv_mul(base, additive)` differ
   in every component. The translation of the result comes from the second
   argument transformed by the first. A reversed order puts the base
   translation through the additive rotation and scale.
*  `additive0` and `additive1`: the translation sum and the scale product
   commute, so only the rotation is at risk.
*  Normalization: compression does not normalize the combined rotation. A
   runtime that normalizes after the combine measures a different error than
   the one compression optimized for. With the `custom` format, the adapter
   must normalize if the runtime normalizes (`transform_base_adapter.h:76-83`).

## 4. Pose equivalence criterion

Two configurations `(clip, base, format, space)` give the same pose if and
only if, for each transform and each sample time, the combined transform in
object space is equal within the tolerance:

```
object_pose = to_object_space(space, apply_additive_to_base(format, base, sample))
```

Apply these rules when you compare:

*  Compare the combined transforms in object space, not the decompressed
   additive samples. Equal additive samples with different bases give different
   poses. Different additive samples with different bases can give the same
   pose.
*  Compare rotations as quaternions up to the sign: `q` and `-q` are the same
   rotation. Compare translations and scales component by component.
*  ACL measures the error on the vertices of a rigid shell around each
   transform, in object space, and keeps the maximum
   (`transform_error_metrics.h:645-676`). Use the same measure for a tolerance
   that agrees with the compression settings: the shell distance and the
   precision of the clip.
*  The exact algebraic check is the round trip:
   `apply_additive_to_base(format, base, convert_to_*(base, x)) == x`. It holds
   up to float rounding when the base rotation has unit length and the base
   scale has no zero component (`convert_to_additive0` divides by the base
   scale, `additive_utils.h:212`).

## 5. Compatibility boundaries

The runtime is compatible with ACL compression when it respects all of these
rules:

1.  Use the same combine function as the `additive_clip_format8` of the clip,
    with the same argument order: the additive transform applies first, the
    base second.
2.  Do not normalize the combined transform, or use the `custom` format with
    an adapter that normalizes.
3.  Use `compressed_tracks::get_default_scale()` for the scale of a constant
    or missing scale sub-track. Do not hardcode 1.0.
4.  Sample the base at the same normalized time as the clip. Compression maps
    each clip sample to the nearest base sample without interpolation. A
    runtime that interpolates the base, or that aligns the two clips by
    absolute time instead of normalized time, gets a different base pose.
5.  Give the base the same transform hierarchy as the clip, in local space.
6.  Keep the base rotation at unit length and the base scale free of zero
    components when you use the `convert_to_*` functions.
7.  With the `custom` format, keep the adapter pure: the same input must
    always give the same output (`transform_base_adapter.h:51-52`). Its
    default scale must be 0.0 or 1.0 (`transform_base.impl.h:81-83`).

Limits that the code enforces:

*  The mesh rotation space rejects the `relative` format.
*  The legacy error metric (`compression_settings::error_metric`) supports
   only the local space and rejects the `custom` format.
*  The compressed data does not store the format. A tool that reads only the
   compressed tracks cannot detect a format mismatch: the check is the
   responsibility of the integration.
