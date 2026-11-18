# Quality tiers, streaming chunks, and fallback decompression

This document analyses how the ACL compression database, the layer that streams the database, and
the fallback decompression path together determine the pose that a runtime can play, and the error
that the pose shows. It also separates two questions that are easy to confuse:

* Does the clip still decompress?
* Is the highest quality data fully present?

Each conclusion gives the source range that confirms it, and the evidence that the source does not
provide.

## 1. Scope and terms

The analysis covers transform tracks only. Scalar decompression refuses a database:
`decompression.scalar.h` returns `false` when the database pointer is not null. Compression exposes
the database switch only for transform tracks (`compression_settings::enable_database_support`).

Terms used below:

* **Highest importance tier (tier 0)**: the samples that stay inside `compressed_tracks`. This tier
  is not part of the database.
* **Medium importance tier (tier 1)**: samples that move into the database, medium bulk data.
* **Lowest importance tier (tier 2)**: samples that move into the database, low bulk data.
* **Chunk**: one addressable unit of database bulk data. A chunk holds whole segments and it cannot
  split a segment.
* **Tier state**: the loaded bit set and the in-flight bit set of one tier, plus the per-segment
  runtime metadata (`tier_metadata`).
* **Playable pose**: the set of transforms that `decompress_tracks` writes for one `seek` call.

`quality_tiers.h` defines three tiers and states that the database holds two of them. The enum
`quality_tier` has `highest_importance = 0`, `medium_importance = 1`, and `lowest_importance = 2`.
`k_num_quality_tiers = 3` and `k_num_database_tiers = 2`.

## 2. The data model that the runtime reads

### 2.1 Tier 0 inside the clip

`build_compressed_tracks` rewrites each input clip. It keeps only the samples of the highest
importance tier in the animated data of each segment. It writes a `stripped_segment_header_t` per
segment. That header holds a 32 bit `sample_indices` mask of the samples that stay in the clip.
It sets the flags `has_database = true` and `has_stripped_keyframes = (num_stripped != 0)`
(`compress.database.impl.h`, `build_compressed_tracks`).

Compression never moves the first sample or the last sample of a segment. `calculate_num_movable_frames`
subtracts two frames for each segment, and `build_database` caps the low and medium tier sizes by
that count (`std::min<uint32_t>(num_movable_frames, ...)`). `assign_frames_to_tier` selects a frame
for the low or medium tier only when its error is strictly below the best error found so far, and
those boundary frames have infinite error. Thus each segment always keeps at least its boundary
samples inside the clip: two samples, or one sample for a clip that has a single sample.
`keyframe_stripping.h` applies the same boundary rule for its own feature, and it asserts it when it
removes a keyframe.

The decode path then asserts the result. `seek_v0` asserts that the closest present sample exists
before and after the requested time, and the comment states the reason: compression never strips the
first or last sample of a segment.

Note that `has_stripped_keyframes` on the rewritten clip reports `(num_stripped != 0)`. A clip whose
tier 0 keeps every sample reports `false`, but `seek_v0` still takes the stripped path because it
computes `has_stripped_keyframes = has_database || tracks->has_stripped_keyframes()`.

### 2.2 Tiers 1 and 2 inside the database

For each tier, the database stores:

* a chunk description list: offset and size of each chunk (`database_header::num_chunks`,
  `get_chunk_descriptions_medium`, `get_chunk_descriptions_low`);
* bulk data. Each chunk starts with a `database_chunk_header` and one
  `database_chunk_segment_header` per segment. Each segment header holds `clip_hash`,
  `sample_indices`, `samples_offset`, `clip_header_offset`, and `segment_header_offset`;
* per-clip runtime headers and per-segment runtime headers. These headers are not bulk data. The
  database context allocates them in one buffer and writes each `clip_hash` at initialize. A
  streamer completion writes the `tier_metadata` values of the segments that its chunks contain.

`database_runtime_segment_header` holds `std::atomic<uint64_t> tier_metadata[2]`. Each value packs
`(sample offset << 32) | sample indices`. The header comment states the reason: a reader can then
load the offset and the indices in lock step while another thread streams data
(`compressed_headers.h`, `database_runtime_segment_header`).

### 2.3 The runtime datum that selects samples

`seek_v0` reads `tier_metadata[0]` and `tier_metadata[1]` for the segment or segments that cover the
sample time. It ORs both masks into the tier 0 `sample_indices` of the clip. It then finds the
closest present sample before the requested time and the closest present sample after it, with bit
scans. It recomputes `interpolation_alpha` from that pair
(`decompression.transform.h`, `seek_v0`, single segment and multiple segment paths).

Thus the pose depends on one datum for each segment: the union of the present sample masks. The
source of a sample, clip or database tier, only selects the data pointer:
`bulk_data_medium + offset`, `bulk_data_low + offset`, or the clip animated data.

The order of the writes matters. In a stream-in completion, `execute_request` caches the bulk data
pointer from `database_streamer::get_bulk_data` before it writes any `tier_metadata`. Thus, after a
completion, a non-zero `tier_metadata` value implies that its tier pointer is set.

A stream out does not keep that order. `stream_out` can set `bulk_data_ref = nullptr` before it
clears `tier_metadata` (`database.impl.h`, `stream_out`). A `seek` at a different time during an
in-flight stream out can then read a non-zero `tier_metadata` value and add its offset to a null
pointer. Section 4.2 item 2 gives the same window from the query side. Section 4.2 item 3 covers the
lifetime of the pointed memory, which the write order does not cover either. The streamer contract
states that a decode and a stream out must not overlap, so a caller that obeys the contract does not
meet this window.

The same order holds after initialization. `relocated` refreshes both tier pointers from the
database header and it does not reset the loaded bits or `tier_metadata` (`database.impl.h`,
`relocated`). The refresh can set a pointer to `nullptr` only when the database no longer stores
inline bulk data. `split_database_bulk_data` rewrites the inline flag, the bulk data offsets, and the
buffer size, so the split database has a different hash. `relocated` then returns `false` for it,
and the caller must initialize the split database with a streamer.

## 3. Part 1: how the three mechanisms interact

### 3.1 Compression assigns frames to tiers by measured error

`build_database` computes `num_low_importance_frames` and `num_medium_importance_frames` from the
proportions in `compression_database_settings`. The remaining frames go to the highest importance
tier. `assign_frames_to_tiers` assigns the lowest tier first, the medium tier second, and the
highest tier last. Each assignment takes the frame with the smallest `stripping_error` across all
clips (`compress.database.impl.h`).

`stripping_error` is measured during quantization. For each candidate frame, compression removes the
frame, interpolates between the nearest retained neighbours, and measures the largest error of the
lost frames. It stores the value in playback units: the value is multiplied by the clip playback
scale (`quantize.transform.h`, `find_contributing_error_impl`, and
`track_array::get_playback_scale`). The first and last frame of a segment get infinite error and are
never selected (`quantize.transform.h`, the `infinity` assignments).

Two preconditions exist. `enable_database_support` forces
`metadata.include_contributing_error = true` during compression
(`compress.transform.impl.h`). `build_database` then rejects a clip without that metadata, a clip
that already has a database, and a clip with stripped keyframes (`compress.database.impl.h`,
`build_database`). Compression also rejects keyframe stripping together with database support
(`compression_settings.impl.h`, `compression_settings::is_valid`).

Consequence: the tier split orders frames by their measured removal error. The order is global
across clips, and it is comparable across clips with different playback scale.

**Limit:** the stored value is an incremental estimate for an ordered removal. The database removes
a whole group of frames at once. The runtime error of a pose therefore is not bounded by the stored
`stripping_error` of the frames that were removed. The comparison asserts in
`validate_database.cpp` use a tolerance and the `>=` direction, not equality, to compare one tier
state with another. Equality is used only to compare a state with itself, for example the
medium tier state before and after a low tier stream in and stream out.

### 3.2 Decode selects the nearest present pair

`seek_v0` does not reconstruct missing samples. It moves the interpolation interval to the nearest
present samples. Two visible effects follow:

* the interpolation alpha grows, and the pose is a wider linear blend;
* the effective rounding policy can become `none`. The code sets
  `context.effective_rounding_policy` when a floor, ceil, or nearest target is absent
  (`decompression.transform.h`, after `find_linear_interpolation_alpha`).

The second effect means that the rounding contract of the API can change when tier data is absent.
`tools/acl_compressor/sources/validate_tracks.cpp` notes the same difference for per track rounding
when keyframes are stripped or the database is partially streamed.

The invariant that makes this path total is the boundary sample rule in 2.1. The code asserts
`candidate_indices0 != 0` and `candidate_indices1 != 0`, and the comment states that compression
never strips the first or last sample of a segment.

### 3.3 Chunks fix the granularity of a tier change

`write_database_bulk_data` writes the bulk data clip by clip and segment by segment. It starts a new
chunk when the next segment does not fit. A segment cannot span two chunks
(`compress.database.impl.h`, with the comment "we can't split a segment over two chunks"). The last
chunk can exceed `max_chunk_size`.

`database_context::stream_in` and `stream_out` always start at the boundary chunk of the loaded set:
`stream_in` at the lowest chunk that is not loaded, and `stream_out` at the lowest chunk that is
loaded. `stream_in` scans the loaded bit set from the lowest word, derives the index from the
trailing zero count, and skips a word with no zero bit. `stream_out` uses the leading zero count
(`database.impl.h`, `stream_in`, `stream_out`). The scan assumes that the loaded chunk set stays a
contiguous run at the low index end. Both APIs act at the low boundary, so a caller cannot create a
hole in the set. For a fresh tier, a partial stream in therefore loads a prefix in clip and segment
order. After a partial stream out, a later partial stream in fills the lowest unloaded chunk first.
Only one stream request can be in flight for each tier. A second request returns
`streaming_in_progress`, or `no_free_streaming_requests` when the streamer has no free request
entry.

A completion updates the per-segment masks. A later `seek` at a different time that covers those
segments sees more samples. Section 4.2 explains why the same time does not.

### 3.4 Conclusions for Part 1

**C1.** A pose is always defined from the samples present in tier 0, plus the samples present in the
database tiers for the segment that covers the requested time. The error of the pose is the result
of the selected sample pair and the interpolation alpha, not of a stored error value.

**C2.** The number of samples present for a segment can only grow when a stream in completes, and it
can shrink when a stream out starts. Quality is therefore a function of tier state, not of time.

**C3.** A tier change happens at chunk granularity, and a chunk holds whole segments. Quality changes
at segment granularity, not at sample granularity.

**C4.** The public API cannot address chunks by clip or by time. A partial stream starts at the
lowest gap in the loaded set, so a fresh tier loads a prefix in clip order. `stream_in` takes a tier
and a chunk count only.

**C5.** A missing sample does not cause an error return. It causes a wider interpolation interval and,
sometimes, a different effective rounding policy. The decompression call itself still succeeds.

## 4. Part 2: three states and what each one delivers

### 4.1 State A: the high importance data is unavailable

Terminology correction first. The highest importance tier is inside `compressed_tracks`. It cannot be
streamed out and it has no database streamer. `database_context` asserts that the tier argument is not
`highest_importance`, and `is_streamed_in` and `is_streaming` return `false` for it.

The state that a caller can meet is one of these:

* **A1: no database context is given.** `decompression_context::initialize(const compressed_tracks&)`
  passes `nullptr`. When database support is compiled in, `has_database` is true and the stripped
  path runs, but every database block is guarded by `db != nullptr` and is skipped. The clip decodes
  from tier 0 only. When database support is disabled, `has_database` is false and the path follows
  `tracks->has_stripped_keyframes()`. `docs/database_support.md` states this case explicitly: "only
  the data within the clip will be used (lowest visual quality)".
* **A2: a database context is given but a tier is not streamed in.** This state applies to the
  streamer overload of `database_context::initialize`. That overload leaves `bulk_data[0]` and
  `bulk_data[1]` null and it does not write `tier_metadata`. The runtime buffer was zeroed
  (`std::memset`), so every `tier_metadata` value is 0. The same decode path then uses tier 0 only.
  The inline overload is different: `initialize(allocator, database)` writes the metadata of every
  chunk and sets the loaded bits for both tiers immediately, so both tiers are present from the
  start.
* **A3: a tier was stripped with `strip_database_quality_tier`.** That tier reports `num_chunks = 0`
  and `has_bulk_data = false`. `stream_in` returns `done` immediately and no data can return.
* **A4: the database does not contain the clip.** This is a hard failure, not a quality drop.
  `decompression_context::initialize(tracks, database)` returns `false` after
  `database.contains(tracks)` fails. The check is part of the initialize safety checks. A caller
  that sets `skip_initialize_safety_checks()` loses it. Decode then indexes the runtime headers with
  the clip header offset from the foreign database, and the result is not defined by the API.

**Still decompresses:** yes for A1, A2, and A3. The tier 0 masks always hold the boundary samples, so
the bit scan always finds a pair. A4 does not decompress with that database.

**Highest quality data fully present:** no for A1, A2, and A3. A1 and A2 can reach full quality later
when both tiers complete a stream in and the caller seeks again. A3 cannot, because the data is
permanently absent.

**Missing evidence:** the source does not state a numeric error for these states. The only numeric
statements are the regression asserts in `validate_database.cpp` and the error thresholds in
`test_data/configs/*_database*.config.sjson`. The assert file is compiled only with
`ACL_USE_SJSON` and `ACL_HAS_ASSERT_CHECKS`, and `validate_db` runs only when the compressor does a
regression test with database support. Both sources apply to the test clips under the test settings.

### 4.2 State B: a chunk completes or leaves

A stream-in request is progressive when the caller passes a chunk count.
`is_streamed_in(tier)` becomes true only when the loaded bit count equals `num_chunks` for that
tier.

Three behaviours are visible in the source:

1. **The seek cache can hide a completed stream-in request.** `seek_v0` returns early when
   `context.sample_time == sample_time` and the requested rounding policy is unchanged
   (`decompression.transform.h`, the early out after the clamp). A stream-in request that completes
   after that seek does not refresh the cached segment headers, data pointers, bit offsets, or alpha.
   The caller must seek to a different time to obtain the new quality.
   `decompression_context::relocated` also forces this refresh because it resets
   `sample_time = -1.0F`.
2. **A stream-out request invalidates data before the bit set changes.** For an asynchronous streamer,
   `stream_out` clears the `tier_metadata` of the affected chunks, and it can set
   `bulk_data_ref = nullptr`, before it calls the streamer. The loaded bit set changes only when the
   streamer completes the request (through `execute_request`). During an in-flight stream out,
   `is_streamed_in` can still return `true` while the metadata for those chunks is already zero. A
   synchronous streamer, such as `null_database_streamer` or `debug_database_streamer`, completes
   inside the call and has no such window.
3. **A stream-out request leaves stale state in the decompression context.** The streamer
   interface states that a stream out must not run while decompression is in progress. The state that `seek` caches
   is a raw pointer into bulk data. After a stream out completes, the context still holds that
   pointer until the next `seek` at a different time. A later `decompress_tracks` without a new
   `seek` therefore reads data that the streamer may have freed.

**Still decompresses:** yes, if the caller obeys the stream-out rule and it seeks again after a
stream out. A stream out lowers quality. It does not remove tier 0 samples.

**Highest quality data fully present:** only when both tiers report `is_streamed_in == true`, no
stream is in flight, and no tier was stripped. `is_streamed_in` alone is not sufficient during an
in-flight stream out, per item 2.

**Missing evidence:** there is no public API that answers "is the data for this clip at this time
fully present". `is_streamed_in` is per tier, not per segment and not per clip. The source also does
not define the pose behaviour when a caller violates the stream out rule; the comment states only
"undefined behavior".

### 4.3 State C: the default data path

The defaults produce a specific, source-confirmed configuration:

* `compression_database_settings` defaults are `medium_importance_tier_proportion = 0.0`,
  `low_importance_tier_proportion = 0.5`, and `max_chunk_size = 1 MB`
  (`compression_settings.h`). `build_database` computes `num_low_importance_frames` as half of the
  total frame count, capped by the movable frame count. Thus the default database has an empty
  medium tier and a low tier with up to half of the frames.
* `decompression_settings::database_settings_type` defaults to `null_database_settings`
  (`decompression_settings.h`). `is_database_supported_impl` then returns `false`, and `seek_v0`
  skips all database metadata reads. With this default, `initialize(tracks, database)` still runs the
  database safety checks (`is_initialized`, `contains`) and it still stores the pointer in the
  context, but the decode path ignores the pointer. When asserts are enabled (`ACL_ON_ASSERT_ABORT`,
  `ACL_ON_ASSERT_THROW`, or `ACL_ON_ASSERT_CUSTOM`), the assert "Cannot have a database when it
  isn't supported" fires. When asserts are stripped (the default), `ACL_ASSERT` becomes
  `((void)0)` (`core/error.h`), so the caller receives tier 0 quality with no diagnostic.
* `compression_settings::is_valid` rejects keyframe stripping together with database support
  (`compression_settings.impl.h`). Keyframe stripping and the database split are separate features
  even though decode handles both through the same `has_stripped_keyframes` condition.
* `get_default_compression_settings()` enables `strip_trivial` keyframe stripping. A caller that
  wants a database must also clear that setting. The compressor tool clears it after it reads a
  config file with `split_into_database` (`acl_compressor.cpp`).

**Still decompresses:** yes. Tier 0 is always in the clip.

**Highest quality data fully present:** the default compression settings put no data in the medium
tier. `is_streamed_in(medium_importance)` is then true with zero chunks, and a medium stream in
returns `done`. Full quality still needs the low tier. A caller who streams only the "medium quality"
tier by default obtains tier 0 quality and a "fully streamed" report for that tier.

**Missing evidence:** the source does not document the empty-medium-tier interaction for a reader.
The behaviour follows from `num_chunks == 0`, `is_streamed_in`'s count comparison, and
`write_database_chunk_descriptions` returning 0 for an empty tier.

### 4.4 Summary table

| State | Decompresses? | Pose source | Full quality available now? |
| --- | --- | --- | --- |
| No database context (A1) | Yes | Tier 0 | No |
| Database bound, streamer overload, tiers idle (A2) | Yes | Tier 0 | No, until a full stream in |
| Tier stripped (A3) | Yes | Tier 0 plus the remaining tier | No, never |
| Clip not in database (A4) | No | None | Not applicable |
| Partial stream-in request (B) | Yes | Tier 0 plus loaded chunks | No, until the last chunk completes |
| Both tiers complete (B) | Yes | Tier 0 plus both tiers | Yes, within the error that `validate_database` measures |
| Default compression settings (C) | Yes | Tier 0 plus the low tier | Only after the low tier completes |

## 5. Part 3: conclusion ledger

Each entry gives the claim, the source range that confirms it, and the evidence that is absent.

### C1: the pose is the union of present sample masks

* **Claim.** The pose comes from the union of the tier 0 mask and the two `tier_metadata` masks for
  the segment that covers the sample time. The error of the pose follows from the selected pair and
  the alpha.
* **Confirmed by.** `seek_v0` in `decompression.transform.h`; `database_runtime_segment_header` in
  `core/impl/compressed_headers.h`.
* **Missing.** No runtime API returns the error of a pose. `track_error` needs the raw tracks and it
  is an author-time measurement.

### C2: quality follows tier state, not time

* **Claim.** A segment has more samples only after a stream-in request completes. It can have fewer
  samples as soon as a stream-out request starts.
* **Confirmed by.** `is_streamed_in`, `stream_in`, `stream_out` in
  `decompression/database/impl/database.impl.h`; `execute_request` in
  `decompression/database/impl/database_streamer.impl.h`.
* **Missing.** No measurement of the error step when one chunk arrives, per clip, per tier size, and
  per content.

### C3: a tier change happens at chunk granularity, and a chunk holds whole segments

* **Claim.** Quality changes at segment granularity. A chunk cannot split a segment.
* **Confirmed by.** `write_database_bulk_data` and `write_database_chunk_descriptions` in
  `compression/impl/compress.database.impl.h`.
* **Missing.** The source does not publish the segment to chunk map. No public API returns it.

### C4: a partial stream loads the lowest unloaded chunks

* **Claim.** A partial stream in always starts at the lowest unloaded chunk, so on a fresh tier it
  loads a prefix in write order. A partial stream out always starts at the lowest loaded chunk. The
  caller cannot select chunks by clip or by time.
* **Confirmed by.** The lowest-gap scan in `stream_in`; the lowest-loaded scan in `stream_out`; the
  write order in `write_database_bulk_data`.
* **Missing.** No evidence that this order is unsuitable for a given integration. The result depends
  on the clip order that the caller supplied to `build_database`.

### C5: a missing sample widens the interval and can change the rounding policy

* **Claim.** Decode still succeeds. The interpolation interval widens, and a floor, ceil, or nearest
  request can become `none`.
* **Confirmed by.** The `effective_rounding_policy` updates and the `candidate_indices` asserts in
  `seek_v0`; the boundary rule in `compression/impl/keyframe_stripping.h` and
  `calculate_num_movable_frames`.
* **Missing.** No statement of the error change for each rounding policy.

### C6: tier assignment orders frames by their measured removal error

* **Claim.** Frames go to tiers in order of increasing measured removal error, in playback units,
  across all clips of the database.
* **Confirmed by.** `find_contributing_error_impl` in `compression/impl/quantize.transform.h`;
  `assign_frames_to_tier` in `compression/impl/compress.database.impl.h`.
* **Missing.** The stored order is incremental and per frame. It is not a runtime bound when a whole
  group of frames is absent at once.

### C7: both tiers complete restores the reference error

* **Claim.** With both tiers streamed in, the error matches the no-database reference within a
  tolerance.
* **Confirmed by.** `validate_database.cpp`, the `high_quality_tier_error` asserts and the
  `medium quality should be restored` asserts.
* **Missing.** The tolerance is specific to the test clips and the platform. It is not a general
  guarantee.

### C8: the error grows as tiers are removed

* **Claim.** Low tier only is worse than medium plus low, and medium plus low is worse than both
  tiers. The database tests assert this order.
* **Confirmed by.** `validate_database.cpp`, the `low >= medium >= high` asserts.
* **Missing.** The mixed-order asserts are commented out with the note "Not guaranteed to always be
  the case due to linear interpolation". The source does not guarantee monotonicity for an arbitrary
  removal order. The tests also skip the whole database validation for clips that contain a zero
  scale, so no evidence covers that class of clip (`validate_db`, `has_zero_scale`).

### C9: the highest importance tier is never streamed

* **Claim.** Tier 0 lives in the clip. No stream request can remove it or add to it.
* **Confirmed by.** The `quality_tier` asserts in `core/compressed_database.h` and
  `decompression/database/database.h`; the tier checks in `database_context`.
* **Missing.** None for transform tracks. Scalar tracks are out of scope because they reject a
  database.

### C10: a stripped tier can never return

* **Claim.** `strip_database_quality_tier` sets `num_chunks = 0` for the tier. `stream_in` then
  returns `done`, so full quality is unreachable.
* **Confirmed by.** `strip_database_quality_tier` in `compression/impl/compress.database.impl.h`;
  `has_bulk_data` and `get_num_chunks` in `core/impl/compressed_database.impl.h`.
* **Missing.** No runtime report that full quality is unreachable. A caller must compare
  `has_bulk_data` with its own requirement.

### C11: a completed stream in does not refresh an unchanged seek

* **Claim.** `seek_v0` returns early for the same time and rounding policy. A completed stream in
  does not update the cached segment headers, data pointers, bit offsets, or alpha.
* **Confirmed by.** The early out in `seek_v0`; the `relocated` function that resets
  `sample_time = -1.0F`.
* **Missing.** No documented contract. The source has no comment that states this consequence.

### C12: `is_streamed_in` can report true during an in-flight stream out

* **Claim.** `stream_out` clears the metadata and can clear the bulk data pointer before it calls
  the streamer. The loaded bit set changes only at completion, so the query can report a tier as
  fully loaded while its metadata is already zero.
* **Confirmed by.** The order of operations in `stream_out`; the bit clear in `execute_request`.
* **Missing.** No test covers this window. The streamer contract forbids a concurrent decode, but it
  does not forbid a concurrent query.

### C13: the default database keeps the medium tier empty

* **Claim.** With `compression_database_settings` defaults, `num_chunks` for the medium tier is 0,
  `is_streamed_in(medium)` is true, and a medium stream in returns `done`.
* **Confirmed by.** The defaults in `compression/compression_settings.h`; the empty-tier return in
  `write_database_chunk_descriptions`; the count comparison in `is_streamed_in`.
* **Missing.** No document states this interaction. The behaviour follows only from those three
  code paths.

### C14: the default decompression settings ignore a database

* **Claim.** With `null_database_settings`, `seek_v0` reads no tier metadata and writes a tier 0
  pose, even when the context was initialized with a database.
* **Confirmed by.** `null_database_settings` in `decompression/database/database_settings.h`;
  `is_database_supported_impl` and `seek_v0` in
  `decompression/impl/decompression.transform.h`.
* **Missing.** With the default assert configuration, there is no diagnostic. Only an enabled
  `ACL_ASSERT` reports the mismatch.

### C15: the database path is transform only

* **Claim.** Scalar decompression rejects a database, and compression exposes the database switch
  only for transform tracks.
* **Confirmed by.** `initialize_v0` and `relocated_v0` in `decompression/impl/decompression.scalar.h`;
  the `enable_database_support` comment in `compression/compression_settings.h`.
* **Missing.** None within the documented API.

### C16: `relocated` is limited to the same database content

* **Claim.** `relocated` refreshes the tier pointers from the database header and it does not reset
  the loaded bits or `tier_metadata`. It accepts the new instance only when the hash matches, and a
  split database has a different hash from its inline source.
* **Confirmed by.** `relocated` in `decompression/database/impl/database.impl.h`;
  `split_database_bulk_data`, which sets the inline flag, the offsets, and the buffer hash.
* **Missing.** The source does not state the required order of `split_database_bulk_data` and
  `relocated` for a caller that moves both buffers. The hash rule shows only that a split database
  must be initialized again.

## 6. Gaps that matter for an integration

1. **No per-pose quality query.** The runtime can report tier state. It cannot report the error of
   the current pose. A caller that must keep a hard error budget needs an author-time table, or its
   own measurement with the raw tracks.
2. **No clip to chunk mapping.** The chunk layout follows clip order and segment order. A caller that
   wants to stream the data of the current clip must order the clips, or accept the prefix order.
3. **No refresh on tier change.** The seek cache answers the same time with the same result.
   Integrations must seek again at a different time after a stream-in request completes.
4. **No partial-tier quality report.** `is_streamed_in` is a whole-tier boolean. It does not say which
   clips or segments are present.
5. **No bound for partial states.** All numeric error evidence comes from `validate_database.cpp` and
   the regression configs, and it covers full-tier states.

## 7. Source index

| File | Relevant content |
| --- | --- |
| `includes/acl/core/quality_tiers.h` | The three tiers and the two database tiers |
| `includes/acl/core/impl/compressed_headers.h` | `stripped_segment_header_t`, `database_runtime_segment_header`, `database_header`, chunk headers |
| `includes/acl/compression/compression_settings.h` | Database proportions, chunk size, and the `enable_database_support` switch |
| `includes/acl/compression/impl/compress.database.impl.h` | Frame counts, tier assignment, clip rewrite, chunk and bulk data write, split, strip |
| `includes/acl/compression/impl/quantize.transform.h` | Contributing error measurement and playback units |
| `includes/acl/compression/impl/keyframe_stripping.h` | Boundary sample rule |
| `includes/acl/decompression/impl/decompression.transform.h` | `seek_v0` and the sample mask union |
| `includes/acl/decompression/database/impl/database.impl.h` | Initialize, relocate, stream in, stream out, `is_streamed_in` |
| `includes/acl/decompression/database/impl/database_streamer.impl.h` | `execute_request` and metadata updates |
| `includes/acl/decompression/database/database_settings.h` | `null_database_settings`, `default_database_settings` |
| `includes/acl/decompression/decompression_settings.h` | `database_settings_type` default and the version constraint |
| `includes/acl/decompression/impl/decompression.scalar.h` | Database rejection for scalar tracks |
| `tools/acl_compressor/sources/validate_database.cpp` | Error ordering and equality asserts for tier states |
| `includes/acl/decompression/database/null_database_streamer.h` | Synchronous in-memory streamer |
| `docs/database_support.md` | User-facing statement of the no-database case |
