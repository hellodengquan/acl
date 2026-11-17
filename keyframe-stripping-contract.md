# Keyframe stripping: selection rule, decompression contract, and acceptance

## 1. Scope

This document analyzes how ACL trades compressed size against animation accuracy when it strips
keyframes. It covers the selection rule, the actual retained boundary, the decompression lookup,
and the split of responsibility between compression and playback. Each claim cites the source.
A claim that comes from reading the code but that no assertion or test enforces is marked
`[inference]`. A claim that depends on a condition outside the code is marked `[dependency]`.

Keyframe stripping applies to transform tracks only (`includes/acl/compression/compression_settings.h:161,
299-301`). The scalar compression path never calls `strip_keyframes`
(`includes/acl/compression/impl/compress.scalar.impl.h` has no such call). The public accessor
`compressed_tracks::has_stripped_keyframes()` asserts a transform track type
(`includes/acl/core/impl/compressed_headers.h:124`, `includes/acl/core/compressed_tracks.h:129-132`).

## 2. Terms

- **Keyframe**: all sub-track samples at one clip sample index
  (`includes/acl/compression/compression_settings.h:141`).
- **Movable keyframe**: a keyframe that compression can remove. The first and last sample of each
  segment are not movable (`includes/acl/compression/impl/keyframe_stripping.h:63-69`,
  `includes/acl/compression/impl/quantize.transform.h:2979-2983`).
- **Stripping error**: the worst error that the removal of one keyframe introduces, measured at the
  source sample times between the two retained neighbors, in playback units
  (`includes/acl/compression/impl/quantize.transform.h:3065-3067`).
- **Stripping order**: the keyframes of the whole clip, sorted from the lowest stripping error to
  the highest (`includes/acl/compression/impl/quantize.transform.h:3143-3188`).

## 3. Settings and the selection rule

`compression_keyframe_stripping_settings` holds three values
(`includes/acl/compression/compression_settings.h:162-208`):

| Value | Meaning | Default |
| --- | --- | --- |
| `proportion` | Minimum fraction of all clip keyframes to strip, in [0, 1] | 0.0 |
| `threshold` | Strip a keyframe when its stripping error is below or equal to this value, in playback units | 0.0 |
| `strip_trivial` | Strip a keyframe when its removal keeps every track below its own precision | false |

Stripping is enabled when any of the three is active (`includes/acl/compression/impl/compression_settings.impl.h:95-98`).
`get_default_compression_settings()` sets `strip_trivial = true`
(`includes/acl/compression/impl/compression_settings.impl.h:193`).

The selection rule has four ordered steps (`includes/acl/compression/impl/keyframe_stripping.h:43-145`):

1. Start with every sample retained (`:60-61`).
2. Compute the number of movable keyframes as `num_samples - 2 * num_segments` when the clip has at
   least two samples, and 0 otherwise (`:69`).
3. Walk the global stripping order and extend the strip count while each entry is trivial
   (`:73-85`). Then continue to walk the same order and extend the strip count while
   `stripping_error <= threshold` (`:87-96`). The scan stops at the first entry above the threshold.
4. Set the strip count to
   `max(count_from_steps_3, min(movable, uint32_t(proportion * num_samples)))`
   (`:98-100`).

Step 4 makes the proportion a floor. It overrides the threshold. The comparison in step 3 uses the
`>` operator, thus a keyframe with an error exactly equal to the threshold is removed. This matches
the setting description (`includes/acl/compression/compression_settings.h:178-185`).

A fifth step cancels the stripping when it does not save memory
(`includes/acl/compression/impl/keyframe_stripping.h:104-121`). The code sums
`animated_pose_bit_size` for each removed keyframe, rounds the total down to bytes, and compares it
with `num_segments * (sizeof(stripped_segment_header_t) - sizeof(segment_header))`. When the saving
is not larger than the overhead, the strip count becomes 0 (`:120-121`). The flag
`has_stripped_keyframes` is set only when the final count is not 0 (`:140`), thus a clip with a
positive proportion can still be written with the normal segment header and the normal
decompression path.

### 3.1 Prerequisites

- Keyframe stripping cannot be combined with `enable_database_support`
  (`includes/acl/compression/impl/compression_settings.impl.h:159-160`). The `acl_compressor` tool
  also clears the settings when it enables the database
  (`tools/acl_compressor/sources/acl_compressor.cpp:1393-1395`).
- At least one of the rotation, translation, and scale formats must be variable. Compression forces
  the contributing error metadata when stripping is enabled, and it then rejects raw tracks with
  `"Raw tracks have no contributing error"`
  (`includes/acl/compression/impl/compress.transform.impl.h:209-218, 220-228`). The tool also
  clears the stripping settings when all formats are raw
  (`tools/acl_compressor/sources/acl_compressor.cpp:1397-1401`).
- A segment holds at most 32 samples, because the sample bitset is 32 bits and the contributing
  error buffer holds 32 entries
  (`includes/acl/compression/impl/compress.transform.impl.h:230-231`,
  `includes/acl/compression/impl/quantize.transform.h:2967-2968, 3134`).
- Compression computes the contributing error only when
  `settings.metadata.include_contributing_error` is true. When stripping is the only reason for that
  metadata, compression removes it after the decision
  (`includes/acl/compression/impl/compress.transform.impl.h:209-218, 372-373`).

The contributing error metadata is a list of `keyframe_stripping_metadata_t`, sorted in stripping
order (`includes/acl/core/impl/compressed_headers.h:393-420, 430-434`;
`includes/acl/compression/impl/quantize.transform.h:3260-3262`).

### 3.2 How the stripping error is measured

For each segment, compression removes the current best candidate repeatedly
(`includes/acl/compression/impl/quantize.transform.h:3006-3090`). For each candidate it does this:

1. Find the nearest retained sample before and after the candidate (`:3022-3030`).
2. Sample the quantized samples at those two times from the bit rate database
   (`:3040-3041`). The database returns the nearest quantized sample
   (`includes/acl/compression/impl/sample_streams.h:560-604`).
3. For every sample between the two neighbors, interpolate in local space with
   `quat_lerp_stable` and `vector_lerp`, then measure the error at that sample time
   (`:3047-3063`).
4. Multiply the worst error by the playback scale and store it (`:3065-3067`).
5. Keep the candidate with the smallest error and remove it (`:3077-3089`).

The measurement uses the lossy sample values from the final bit rates, and it uses the same time
formula as decompression: `min(index / sample_rate, clip_duration)` (`:3035-3036, 3067`). The
trivial flag is set only when every track stays below its own precision at every tested sample
(`:2847-2853, 2947-2951`). Compression always measures with scale to keep the result consistent
(`:3125-3129`).

Consequence: the recorded error is the error of one candidate removal, measured with the neighbors
that exist at that moment. It is not recomputed after later removals.

## 4. The actual retained boundary

After the selection rule, the retained set is the complement of a prefix of the global stripping
order. This has these properties.

- **Per segment, the first and last samples always stay.** They are never candidates
  (`includes/acl/compression/impl/quantize.transform.h:2979-2983`), the removal loop asserts this
  (`includes/acl/compression/impl/keyframe_stripping.h:130`), and decompression relies on it
  (`includes/acl/decompression/impl/decompression.transform.h:734-743, 898-907`). Thus the
  boundary density increases at every segment split, which by default happens about every 16
  samples with a maximum of 31 (`includes/acl/compression/impl/segment_context.h:55-80`).
- **The removals are global, not uniform.** `sort_contributing_error` merges the per-segment
  orders by repeatedly taking the lowest error over all segments, and it prefers the later segment
  on a tie (`includes/acl/compression/impl/quantize.transform.h:3156-3185`). Quiet or slow
  sections therefore lose more keyframes than fast sections. Contacts and motion extrema usually
  have a high error, thus they are the last candidates.
- **The realized fraction of movable keyframes is higher than `proportion`.** The proportion
  multiplies the total sample count, which includes the immovable samples
  (`includes/acl/compression/impl/keyframe_stripping.h:99`). On a 30-sample single-segment clip,
  `proportion = 0.5` requests 15 removals out of 28 movable samples.
- **`proportion = 1.0` does not empty the clip.** The request is clamped to the movable count, so
  each segment keeps its first and last sample
  (`includes/acl/compression/impl/keyframe_stripping.h:99-100`).
- **The result can be no stripping at all**, because of the memory gate
  (`includes/acl/compression/impl/keyframe_stripping.h:104-121`). The gate counts only animated
  bits. Constant and default sub-tracks cost nothing per sample, so a clip with few animated bits
  can fail the gate.

The corresponding on-disk state is one `stripped_segment_header_t` per segment, which adds a 32-bit
`sample_indices` bitset after the normal segment header
(`includes/acl/core/impl/compressed_headers.h:228-237`). Compression writes the bitset from
`segment.hard_keyframes` and skips the removed samples when it packs the animated data
(`includes/acl/compression/impl/write_segment_data.h:80-98`,
`includes/acl/compression/impl/write_stream_data.h:465-468, 495-502`).

Worked example: 60 samples, 4 segments of 15 samples, threshold 0.0, proportion 0.6.
`num_keyframes_to_strip` starts at `min(52, uint32_t(0.6 * 60)) = 36`. The 36 keyframes with the
lowest stripping error are removed. Samples 0, 14, 15, 29, 30, 44, 45, and 59 always stay, plus the
other 16 keyframes that the global order leaves below the cutoff. Without the threshold the 36
removals can include keyframes with a large error, because step 4 overrides step 3.

## 5. Does the proportion promise the error bound at every sample time?

No. Four separate points support this answer.

1. **The proportion is a floor, not a cap.** After the threshold scan stops, step 4 raises the count
   to the proportion value (`includes/acl/compression/impl/keyframe_stripping.h:98-100`). The
   keyframes that the proportion adds can have a stripping error above the threshold. No code
   rejects them, and their measured error is not reported unless the caller asks for the optional
   metadata.
2. **The threshold is not the precision of the tracks.** `threshold` is one clip-wide value in
   playback units (`includes/acl/compression/compression_settings.h:178-185`). Each track has its
   own `precision` (`includes/acl/core/track_desc.h:117-123`). The threshold bounds the worst
   object-space shell error of a removal. It does not bound the error of each track separately.
3. **Even a threshold-only configuration does not measure between source sample times.**
   `find_contributing_error_impl` evaluates only integer sample indices between the retained
   neighbors (`includes/acl/compression/impl/quantize.transform.h:3047-3063`). The public error
   function `calculate_compression_error` also evaluates only integer sample indices
   (`includes/acl/compression/impl/track_error.impl.h:352-358`). No code in the library measures a
   time that falls between two source samples.
4. **The noise floor of the quantization search is best effort.** The bit rate search uses the
   precision as a target, and it keeps the best permutation when no permutation reaches the target
   (`includes/acl/compression/impl/quantize.transform.h:1341-1346, 2530-2534`). Thus a retained
   sample is not guaranteed to satisfy its precision. `[dependency]` The actual error depends on
   the bit rates that the search selects for the clip.

### 5.1 What the threshold does bound `[inference]`

Assume that the caller sets `proportion = 0` and does not enable `strip_trivial`, so the strip set
is exactly the prefix whose stripping error is at most the threshold. Then, for every removed source
sample time, the final reconstruction error is at most the threshold. Reason: for every retained
interval, the last removal in that interval used the final two neighbors as its interpolation
endpoints, and its measurement covered every sample in the interval
(`includes/acl/compression/impl/quantize.transform.h:3022-3063`). That last removal belongs to the
removed prefix, so its error is at most the threshold. The endpoints that the measurement takes from
the bit rate database are the samples that the stream stores, because compression finds the
contributing error after it selects the bit rates and before it quantizes the streams
(`includes/acl/compression/impl/quantize.transform.h:3246-3257`). No assertion in the code states
this property, and no test asserts it. It also depends on the metric that compression uses being the
metric that the runtime shows (`[dependency]`).

### 5.2 Is the bound valid between sample times?

For translation and scale, both the source model and the reconstruction are linear between their
own breakpoints, and the breakpoints are source sample times
(`includes/acl/compression/impl/track_array.impl.h:344-346`,
`includes/acl/decompression/impl/animated_track_cache.transform.h:1612-1637`). The difference
between the two is linear on each sub-interval, therefore the largest difference is attained at a
breakpoint, which is a source sample time. `[inference]` Under the conditions of section 5.1, the
threshold then holds at every time for translation and scale.

For rotations, the result does not follow, because the interpolation is normalized after the linear
step (`includes/acl/math/quatf.h:380-399`,
`includes/acl/decompression/impl/animated_track_cache.transform.h:1552-1561`). Normalization is
nonlinear, so the maximum error can occur between samples, and it can be larger than the maximum at
the sample times. `[inference]` The excess is not quantified by ACL.

Also, the runtime model itself is piecewise linear between authored samples. Motion that happens
between authored samples, such as a contact that occurs at half a frame, is not in the source data.
No compression setting can preserve it, and no error measured against the source can show it.

## 6. Precision at source sample times and between sample times

Keep the two budgets separate during design and during acceptance.

**At an authored sample time that is retained.** The pose is the quantized sample of that keyframe.
Its error is the compression error of that sample. The value is best effort against the per-track
precision (section 5, item 4). Source code: the sample is unpacked directly from its bit offset
(`includes/acl/decompression/impl/decompression.transform.h:1012-1013`).

**At an authored sample time that is removed.** The pose is reconstructed by interpolation of the
two surrounding retained samples. The interpolation happens in local space, before the runtime
composes the object-space pose. Source code: `find_contributing_error_impl` interpolates local
transforms (`includes/acl/compression/impl/quantize.transform.h:3052-3060`), and decompression
interpolates the same local samples (`includes/acl/decompression/impl/animated_track_cache.transform.h:1552-1637`).
The measured error is the object-space shell error, which includes the parent contribution.

**Between two authored sample times.** The pose comes from the same interpolation, but over a
longer interval when a keyframe was removed. Compared with the source linear model, the bound of
section 5.2 holds for translation and scale, and it does not hold for rotation.
`[dependency]` This assumes that the runtime keeps the default rotation normalization policy
`lerp_only` (`includes/acl/decompression/decompression_settings.h:52-62, 239`). With
`rotation_normalization_policy_t::never`, decompression takes the unnormalized branch
(`includes/acl/decompression/impl/animated_track_cache.transform.h:1556-1561`), and the
reconstruction no longer matches the function that compression measured.

**Between two authored sample times with a rounding policy.** `floor`, `ceil`, and `nearest` ask for
a specific sample. When that sample was removed, the library downgrades the effective policy to
interpolation (`includes/acl/decompression/impl/decompression.transform.h:750-758, 918-926`). The
requested pose is then the interpolated pose, not the authored pose. `per_track` is not upgraded
this way; the documentation states that only the interpolated value is available, so the result
differs from `floor` and `ceil` without stripping
(`includes/acl/core/sample_rounding_policy.h:81-108`). The regression validator skips the comparison
of `per_track` against the other policies for a stripped clip
(`tools/acl_compressor/sources/validate_tracks.cpp:211-226`).

## 7. How decompression finds the retained samples

### 7.1 Step by step

1. `seek` maps the requested time to clip keyframes and an alpha with
   `find_linear_interpolation_samples_with_sample_rate` and the requested rounding policy
   (`includes/acl/decompression/impl/decompression.transform.h:629-663`).
2. The clip takes the stripped path when it has a database or when
   `has_stripped_keyframes()` is true (`:688-691`).
3. Compression guarantees that the first and last sample of each segment survive (section 4), and
   decompression asserts this (`:734-743, 898-907`).
4. For the single-segment case, decompression loads `sample_indices`, ORs the database tier bitsets
   when a database is present, then finds:
   - the closest retained sample at or before `clip_key_frame0`, with a mask and
     `count_trailing_zeros` (`:734-737`);
   - the closest retained sample at or after `clip_key_frame1`, with a mask and
     `count_leading_zeros` (`:740-743`).

   The bitset stores sample 0 in bit 31 (`includes/acl/core/bitset.h:135-147`). Thus a search result
   is the retained sample nearest to the requested time on each side. Search cost is two integer
   operations on one 32-bit word per segment, which is the source of the fixed overhead that the
   header describes as below 200 instructions
   (`includes/acl/compression/compression_settings.h:148-151`).
5. Decompression recomputes the alpha between the two retained samples from the original fractional
   index, with `sample_rounding_policy::none` (`:745-748`). The fractional position is preserved, so
   the result keeps the phase of the requested time.
6. Decompression remaps each retained sample index to a dense index with
   `count_set_bits(and_not(mask, sample_indices))` (`:795-797, 962-964`). The dense index multiplied
   by `animated_pose_bit_size` gives the bit offset of the sample in its segment data
   (`:1012-1013`). Compression produced the stream with the removed samples omitted
   (`includes/acl/compression/impl/write_stream_data.h:465-468`), so the dense index is correct.
7. The animated cache unpacks the sample at each bit offset and interpolates the two local poses
   (`includes/acl/decompression/impl/animated_track_cache.transform.h:1136-1138, 1552-1573, 1612-1637`).

The multi-segment path adds a segment search
(`includes/acl/decompression/impl/decompression.transform.h:812-975`). It searches a window around
the estimated segment, then applies the same two searches to the bitset of each segment that the two
key frames belong to. When the two key frames fall in different segments, the middle of the
interpolation can be the segment boundary. This works because the boundary samples are always
retained.

### 7.2 Runtime observations

- The effective policy can differ from the requested policy. The internal context stores and
  exposes it (`includes/acl/decompression/impl/decompression_context.transform.h:109`). The public
  `decompression_context` wrapper does not expose it
  (`includes/acl/decompression/decompress.h:96-172`). `[inference]` Client code that needs to know
  the effective policy cannot read it through the public API in this revision.
- A request for a sample time does not guarantee the authored pose of that time. It guarantees the
  pose of the interpolation that uses the retained neighbors.
- With the wrap policy, the alpha computation uses the wrap case
  (`includes/acl/core/impl/interpolation_utils.impl.h:225-254`). The first sample is always retained,
  so the wrap boundary is safe.

## 8. Responsibility split

| Decision | Compression owns | Playback owns |
| --- | --- | --- |
| Which keyframes exist | Selects the strip set, writes `sample_indices`, keeps first and last per segment | Does not assume that any authored keyframe exists |
| Error at authored sample times | Measures it with the configured runtime transform type and metric, applies the threshold and the trivial rule | Uses `none` when accuracy matters, or accepts the downgrade |
| Error between sample times | Does not measure it | Owns the check, with dense sampling |
| Segment layout | Splits into segments of about 16 samples, maximum 31 | Uses the segment that holds each retained sample |
| Contact and sync behavior | Cannot know gameplay contacts; the header lists contacts, apex, and sync as known problems (`includes/acl/compression/compression_settings.h:153-160`) | Owns IK, ground snapping, and any gameplay correction |
| Playback scale | Divides the precision and the contributing error by the scale | Uses the same scale when it measures the error; the compressed data does not store the scale (`includes/acl/compression/track_array.h:143-159`) |

Two consequences follow. First, compression cannot promise a contact at a time that the source does
not sample, and it cannot recognize a contact as special. Second, the runtime cannot recover the
authored pose of a removed keyframe; it can only interpolate or correct the pose with its own logic.

## 9. Settings for contact points and motion extrema

The audio-visual quality target is a per-clip choice. The following configurations follow from the
code above.

### 9.1 Contact-heavy clips

1. Set `proportion = 0.0` and use the threshold only. A floor forces removals above the threshold
   (section 5, item 1).
2. Set `threshold` to the largest contact error that the game accepts, in playback units. Keep it at
   or below the per-track precision if the contact depends on one transform. Use a lower threshold
   for clips that need exact foot placement.
3. Keep `strip_trivial` if the runtime does not use `per_track` rounding. Trivial stripping adds a
   per-track precision check (`includes/acl/compression/impl/quantize.transform.h:2847-2853`). Turn
   it off when `per_track` is in use, as the settings comment requires
   (`includes/acl/compression/compression_settings.h:187-198`). Note that the trivial check compares
   each track against its own precision, not against `threshold`. A trivial keyframe can therefore
   exceed a small `threshold`. Turn `strip_trivial` off when the threshold must bound the error.
4. Use `sample_rounding_policy::none` for contact queries. Do not use `floor` or `nearest` to hold a
   contact, because the requested sample can be absent and the policy then becomes interpolation
   (`includes/acl/decompression/impl/decompression.transform.h:750-758`).
5. Author a keyframe at each contact, and if possible place it at a segment boundary. The boundary
   samples are always retained, and the boundary is about every 16 samples by default.
6. Use the same settings for clips that must stay synchronized. The header warns that uneven
   removal between synchronized clips causes missed contacts
   (`includes/acl/compression/compression_settings.h:157-159`).
7. If a hard contact guarantee is required, keep stripping off for that clip and let the runtime use
   IK. No setting in ACL makes a removed keyframe exact.

### 9.2 Clips with motion extrema

1. Use the threshold only. Extrema usually have a high stripping error, thus the global order
   already places them last (section 4). A proportion floor can still remove them.
2. Keep the `strip_trivial` default only after the measurement in section 10. A trivial keyframe is
   reconstructed below the per-track precision at its own sample time, which is usually safe at an
   extremum only if the precision is small enough for the visible amplitude.
3. Watch the velocity. The header names "swimming" from lost momentum as a known problem
   (`includes/acl/compression/compression_settings.h:154-156`). Measure the velocity around the
   extremum in acceptance, not only the position.
4. Author a denser sample set around impacts and apexes. ACL cannot preserve a peak that falls
   between samples.

### 9.3 Reference configuration

The regression configuration for stripping sets `proportion = 0.0` and `threshold = 0.5`, with the
variable formats and the Medium level
(`test_data/configs/uniformly_sampled_keyframe_stripping.config.sjson:14-15`). This is a size and
quality reference, not a contact guarantee.

## 10. Acceptance

Acceptance must test retention and interpolation separately.

### 10.1 Retention at source sample times

- Enable the detailed statistics build. The statistics write `num_stripped_keyframes` and
  `num_trivial_keyframes` for each segment
  (`includes/acl/compression/impl/write_stats.h:82-97`).
- For every strip candidate, seek exactly at its sample time with `sample_rounding_policy::none`,
  and compare the pose with the raw pose at the same time. Require the error to be at most the
  threshold when `proportion = 0`.
- Check the contact bones by name, not only the clip-wide maximum. The threshold is one value for
  the clip, and a contact can use a precise track whose error is hidden by the maximum.
- Check that `has_stripped_keyframes()` matches the expectation
  (`includes/acl/core/compressed_tracks.h:129-132`). A size regression with the flag clear means the
  memory gate rejected the stripping (`includes/acl/compression/impl/keyframe_stripping.h:104-121`).
- For a clip that uses `per_track` rounding, do not compare it with the other policies
  (`tools/acl_compressor/sources/validate_tracks.cpp:211-226`). Test it with `none` instead.

### 10.2 Interpolation between sample times

- Sample the compressed clip and the raw track array at a denser time step than the sample rate,
  for example 4 times to 8 times the rate, with `none` on both sides. This uses
  `context.seek` plus `raw_tracks.sample_tracks` (`includes/acl/compression/impl/track_error.impl.h:352-358`
  shows the integer-time form that the library itself uses).
- Split the report into two numbers: the error at authored sample times, and the extra error at
  sub-sample times. Do not merge them, because section 5 shows that they have different guarantees.
- For rotations, expect small sub-sample excesses from the normalization
  (`includes/acl/math/quatf.h:380-399`). Set a separate budget for them.
- Check the contact windows and the extrema windows with the velocity or position of the affected
  bones.

## 11. Evidence gaps and dependencies

- `[inference]` The prefix argument in section 5.1. No assertion or test states it.
- `[inference]` The piecewise-linear argument for translation and scale in section 5.2.
- `[dependency]` Compression and playback must use the same runtime transform type, metric, and
  rotation normalization policy. Otherwise the measured stripping error is not the error that the
  runtime shows (`includes/acl/decompression/impl/animated_track_cache.transform.h:1552-1561`,
  `includes/acl/decompression/decompression_settings.h:52-62, 239`).
- `[dependency]` The per-track precision must be at or below the desired error. The bit rate search
  is best effort (`includes/acl/compression/impl/quantize.transform.h:1341-1346`).
- `[dependency]` The playback scale must be at least as large as the largest scale at playback,
  because compression divides the precision and the threshold by it
  (`includes/acl/compression/track_array.h:143-159`).
- Not verified here by execution: the numeric size and error of a strip set on the regression clips.
  The unit test checks only that a 0.5 proportion yields a smaller clip and a higher error
  (`tests/sources/compression/test_transform_runtime_compression.cpp:405-431`).

## 12. Source index

| Topic | Location |
| --- | --- |
| Settings and warnings | `includes/acl/compression/compression_settings.h:139-208` |
| Enable, validate, defaults | `includes/acl/compression/impl/compression_settings.impl.h:95-119, 193` |
| Selection rule | `includes/acl/compression/impl/keyframe_stripping.h:43-145` |
| Strip order metadata | `includes/acl/core/impl/compressed_headers.h:393-420` |
| Stripping error measurement | `includes/acl/compression/impl/quantize.transform.h:2962-3095, 3143-3188` |
| Trivial test | `includes/acl/compression/impl/quantize.transform.h:2847-2853, 2947-2951` |
| Metadata forced and removed | `includes/acl/compression/impl/compress.transform.impl.h:209-218, 372-373` |
| Segment header with bitset | `includes/acl/core/impl/compressed_headers.h:228-237` |
| Bitset written | `includes/acl/compression/impl/write_segment_data.h:80-98` |
| Samples omitted from the stream | `includes/acl/compression/impl/write_stream_data.h:465-468` |
| Seek lookup | `includes/acl/decompression/impl/decompression.transform.h:629-1017` |
| Interpolation | `includes/acl/decompression/impl/animated_track_cache.transform.h:1136-1138, 1552-1573, 1612-1637` |
| Rotation lerp | `includes/acl/math/quatf.h:380-399` |
| Policy semantics | `includes/acl/core/sample_rounding_policy.h:44-108` |
| Error units | `includes/acl/compression/track_error.h:53-67` |
| Error sampling times | `includes/acl/compression/impl/track_error.impl.h:352-358` |
| Detail statistics | `includes/acl/compression/impl/write_stats.h:82-97` |
| Regression configuration | `test_data/configs/uniformly_sampled_keyframe_stripping.config.sjson:14-15` |
