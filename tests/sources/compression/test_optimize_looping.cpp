////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2026 Nicholas Frechette & Animation Compression Library contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include "catch2.impl.h"

#include <acl/compression/compress.h>
#include <acl/compression/track_array.h>
#include <acl/core/ansi_allocator.h>
#include <acl/core/impl/debug_track_writer.h>
#include <acl/decompression/decompress.h>

#include <rtm/quatf.h>
#include <rtm/scalarf.h>
#include <rtm/vector4f.h>

#include <cmath>
#include <cstdint>
#include <utility>

using namespace acl;
using namespace acl::acl_impl;

namespace
{
	constexpr uint32_t k_num_transforms = 2;
	constexpr uint32_t k_num_samples = 5;
	constexpr float k_sample_rate = 4.0F;
	constexpr float k_precision = 0.01F;

	// The duration of a clip with k_num_samples samples: (k_num_samples - 1) / k_sample_rate = 1.0
	constexpr float k_duration = float(k_num_samples - 1) / k_sample_rate;

	// The last sample matches the first sample within the precision
	constexpr float k_loop_offset = 0.003F;

	// The last sample differs from the first sample by more than the precision
	constexpr float k_no_loop_offset = 1.0F;

	// The loop detection and the quantization both use the precision as their error budget
	constexpr float k_error_budget = k_precision * 2.0F;

	track_desc_transformf make_transform_desc(uint32_t transform_index)
	{
		track_desc_transformf desc;
		desc.output_index = transform_index;
		desc.parent_index = transform_index == 0 ? k_invalid_track_index : 0;
		desc.precision = k_precision;
		desc.shell_distance = 1.0F;
		return desc;
	}

	// A clip with a root transform and its child.
	// The root rotates around the Z axis and each transform translates along the X axis.
	// Sample I holds the value I on the X axis and the last sample holds 'last_offset'.
	track_array_vqvf make_transform_clip(iallocator& allocator, uint32_t num_samples, float last_offset)
	{
		track_array_vqvf track_list(allocator, k_num_transforms);

		for (uint32_t transform_index = 0; transform_index < k_num_transforms; ++transform_index)
		{
			track_vqvf track = track_vqvf::make_reserve(make_transform_desc(transform_index), allocator, num_samples, k_sample_rate);
			for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
			{
				const bool is_last_sample = sample_index + 1 == num_samples;
				const float translation_x = is_last_sample ? last_offset : float(sample_index);
				const float rotation_angle = is_last_sample || transform_index != 0 ? 0.0F : 0.2F * float(sample_index);
				track[sample_index].rotation = rtm::quat_from_axis_angle(rtm::vector_set(0.0F, 0.0F, 1.0F), rotation_angle);
				track[sample_index].translation = rtm::vector_set(translation_x, 0.5F * float(transform_index), 0.0F);
				track[sample_index].scale = rtm::vector_set(1.0F);
			}

			track_list[transform_index] = std::move(track);
		}

		return track_list;
	}

	// A clip where every sample holds the same pose
	track_array_vqvf make_static_transform_clip(iallocator& allocator, uint32_t num_samples)
	{
		track_array_vqvf track_list(allocator, k_num_transforms);

		for (uint32_t transform_index = 0; transform_index < k_num_transforms; ++transform_index)
		{
			track_vqvf track = track_vqvf::make_reserve(make_transform_desc(transform_index), allocator, num_samples, k_sample_rate);
			for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
			{
				track[sample_index].rotation = rtm::quat_identity();
				track[sample_index].translation = rtm::vector_set(1.5F, 0.5F * float(transform_index), 0.0F);
				track[sample_index].scale = rtm::vector_set(1.0F);
			}

			track_list[transform_index] = std::move(track);
		}

		return track_list;
	}

	// A scalar clip with a single track.
	// Sample I holds the value I and the last sample holds 'last_offset'.
	track_array_float1f make_scalar_clip(iallocator& allocator, uint32_t num_samples, float last_offset)
	{
		track_desc_scalarf desc;
		desc.output_index = 0;
		desc.precision = k_precision;

		track_array_float1f track_list(allocator, 1);

		track_float1f track = track_float1f::make_reserve(desc, allocator, num_samples, k_sample_rate);
		for (uint32_t sample_index = 0; sample_index < num_samples; ++sample_index)
		{
			const bool is_last_sample = sample_index + 1 == num_samples;
			track[sample_index] = is_last_sample ? last_offset : float(sample_index);
		}

		track_list[0] = std::move(track);
		return track_list;
	}

	compressed_tracks* compress_clip(iallocator& allocator, const track_array& track_list, bool optimize_loops)
	{
		compression_settings settings = get_default_compression_settings();
		settings.optimize_loops = optimize_loops;
		settings.keyframe_stripping.strip_trivial = false;	// Keep every sample

		output_stats stats;
		compressed_tracks* tracks = nullptr;
		const error_result result = compress_track_list(allocator, track_list, settings, tracks, stats);
		REQUIRE(result.empty());
		REQUIRE(tracks != nullptr);
		REQUIRE(tracks->is_valid(true).empty());
		return tracks;
	}

	// Returns true when the angle between two rotations is within the threshold
	bool is_rotation_near(rtm::quatf_arg0 expected, rtm::quatf_arg1 actual, float angle_threshold)
	{
		// The dot product of two unit quaternions is the cosine of half the angle between them
		const float dot = rtm::scalar_cast(rtm::quat_dot_as_scalar(expected, actual));
		return rtm::scalar_abs(dot) >= std::cos(angle_threshold * 0.5F);
	}

	float get_translation_x(const rtm::vqvf& pose)
	{
		return rtm::vector_get_x(pose.translation);
	}
}

TEST_CASE("loop optimization wraps a looping transform clip", "[compression][transform][loop]")
{
	ansi_allocator allocator;

	track_array_vqvf raw_tracks = make_transform_clip(allocator, k_num_samples, k_loop_offset);
	REQUIRE(raw_tracks.get_num_samples_per_track() == k_num_samples);
	const float raw_duration = raw_tracks.get_duration();
	REQUIRE(rtm::scalar_near_equal(raw_duration, k_duration, 1.0E-6F));

	compressed_tracks* tracks = compress_clip(allocator, raw_tracks, true);

	// The last sample matches the first sample: it is removed and the wrap policy applies
	CHECK(tracks->get_looping_policy() == sample_looping_policy::wrap);
	CHECK(tracks->get_num_samples_per_track() == k_num_samples - 1);

	// The removed sample repeats the first sample: the playback duration stays the same
	CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::as_compressed), raw_duration, 1.0E-6F));
	CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::wrap), raw_duration, 1.0E-6F));

	// The same data with the clamp policy is one frame shorter
	CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::clamp), raw_duration - (1.0F / k_sample_rate), 1.0E-6F));

	decompression_context<debug_transform_decompression_settings> context;
	REQUIRE(context.initialize(*tracks));
	CHECK(context.get_looping_policy() == sample_looping_policy::wrap);

	debug_track_writer writer(allocator, track_type8::vqvf, k_num_transforms);

	const rtm::quatf identity = rtm::quat_identity();

	// The start of the clip holds the first sample
	context.seek(0.0F, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 0.0F, k_error_budget));
	CHECK(is_rotation_near(identity, writer.read_vqv(0).rotation, 0.05F));

	// The middle of the clip holds an exact sample
	context.seek(0.5F, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 2.0F, k_error_budget));

	// The end of the clip wraps to the first sample: it matches the start of the clip
	context.seek(k_duration, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 0.0F, k_error_budget));
	CHECK(is_rotation_near(identity, writer.read_vqv(0).rotation, 0.05F));

	// The raw last sample differs from the first sample by k_loop_offset:
	// the output at the end stays within the precision budget of the raw last sample
	CHECK(std::fabs(get_translation_x(writer.read_vqv(0)) - k_loop_offset) <= k_error_budget);

	// Half a frame before the end, the clip interpolates the last stored sample and the first sample.
	// A clip that clamps or that plays one frame less would return the last stored sample (3.0).
	context.seek(k_duration - (0.5F / k_sample_rate), sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 1.5F, k_error_budget));
	CHECK(is_rotation_near(rtm::quat_from_axis_angle(rtm::vector_set(0.0F, 0.0F, 1.0F), 0.3F), writer.read_vqv(0).rotation, 0.05F));
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(1)), 1.5F, 0.05F));

	allocator.deallocate(tracks, tracks->get_size());
}

TEST_CASE("loop optimization keeps every sample of a looping transform clip when disabled", "[compression][transform][loop]")
{
	ansi_allocator allocator;

	track_array_vqvf raw_tracks = make_transform_clip(allocator, k_num_samples, k_loop_offset);
	compressed_tracks* tracks = compress_clip(allocator, raw_tracks, false);

	// The optimization is off: every sample is retained and the clamp policy applies
	CHECK(tracks->get_looping_policy() == sample_looping_policy::clamp);
	CHECK(tracks->get_num_samples_per_track() == k_num_samples);
	CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::as_compressed), k_duration, 1.0E-6F));

	decompression_context<debug_transform_decompression_settings> context;
	REQUIRE(context.initialize(*tracks));
	CHECK(context.get_looping_policy() == sample_looping_policy::clamp);

	debug_track_writer writer(allocator, track_type8::vqvf, k_num_transforms);

	// The end of the clip holds the true last sample
	context.seek(k_duration, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), k_loop_offset, k_error_budget));

	// Half a frame before the end, the clip interpolates the last two samples
	context.seek(k_duration - (0.5F / k_sample_rate), sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 0.5F * (3.0F + k_loop_offset), k_error_budget));

	allocator.deallocate(tracks, tracks->get_size());
}

TEST_CASE("loop optimization keeps the last sample of a non-looping transform clip", "[compression][transform][loop]")
{
	ansi_allocator allocator;

	// The last sample differs from the first sample by more than the precision: the clip does not loop
	track_array_vqvf raw_tracks = make_transform_clip(allocator, k_num_samples, k_no_loop_offset);
	compressed_tracks* tracks = compress_clip(allocator, raw_tracks, true);

	CHECK(tracks->get_looping_policy() == sample_looping_policy::clamp);
	CHECK(tracks->get_num_samples_per_track() == k_num_samples);
	CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::as_compressed), k_duration, 1.0E-6F));

	decompression_context<debug_transform_decompression_settings> context;
	REQUIRE(context.initialize(*tracks));

	debug_track_writer writer(allocator, track_type8::vqvf, k_num_transforms);

	// The end of the clip holds the true last sample: the difference with the first sample is retained
	context.seek(k_duration, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), k_no_loop_offset, k_error_budget));

	// Half a frame before the end, the clip interpolates the last two samples and does not wrap
	context.seek(k_duration - (0.5F / k_sample_rate), sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 0.5F * (3.0F + k_no_loop_offset), k_error_budget));

	allocator.deallocate(tracks, tracks->get_size());
}

TEST_CASE("loop optimization keeps a transform clip with a single pose", "[compression][transform][loop]")
{
	ansi_allocator allocator;

	// A single sample cannot wrap
	track_array_vqvf raw_tracks = make_transform_clip(allocator, 1, 2.5F);
	compressed_tracks* tracks = compress_clip(allocator, raw_tracks, true);

	CHECK(tracks->get_looping_policy() == sample_looping_policy::clamp);
	CHECK(tracks->get_num_samples_per_track() == 1);
	CHECK(tracks->get_finite_duration(sample_looping_policy::as_compressed) == 0.0F);

	decompression_context<debug_transform_decompression_settings> context;
	REQUIRE(context.initialize(*tracks));

	debug_track_writer writer(allocator, track_type8::vqvf, k_num_transforms);

	context.seek(0.0F, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 2.5F, k_error_budget));

	allocator.deallocate(tracks, tracks->get_size());
}

TEST_CASE("loop optimization wraps a transform clip with a repeated pose", "[compression][transform][loop]")
{
	ansi_allocator allocator;

	// Every sample holds the same pose: the last sample matches the first sample
	track_array_vqvf raw_tracks = make_static_transform_clip(allocator, k_num_samples);
	compressed_tracks* tracks = compress_clip(allocator, raw_tracks, true);

	CHECK(tracks->get_looping_policy() == sample_looping_policy::wrap);
	CHECK(tracks->get_num_samples_per_track() == k_num_samples - 1);
	CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::as_compressed), k_duration, 1.0E-6F));

	decompression_context<debug_transform_decompression_settings> context;
	REQUIRE(context.initialize(*tracks));

	debug_track_writer writer(allocator, track_type8::vqvf, k_num_transforms);

	// Every point in time holds the same pose
	const float sample_times[] = { 0.0F, 0.3F, k_duration - (0.5F / k_sample_rate), k_duration };
	for (float sample_time : sample_times)
	{
		context.seek(sample_time, sample_rounding_policy::none);
		context.decompress_tracks(writer);
		CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 1.5F, k_error_budget));
	}

	allocator.deallocate(tracks, tracks->get_size());
}

TEST_CASE("decompression overrides the looping policy of a wrapped transform clip", "[compression][transform][loop]")
{
	ansi_allocator allocator;

	track_array_vqvf raw_tracks = make_transform_clip(allocator, k_num_samples, k_loop_offset);
	compressed_tracks* tracks = compress_clip(allocator, raw_tracks, true);
	REQUIRE(tracks->get_looping_policy() == sample_looping_policy::wrap);

	decompression_context<debug_transform_decompression_settings> context;
	REQUIRE(context.initialize(*tracks));
	CHECK(context.get_looping_policy() == sample_looping_policy::wrap);

	debug_track_writer writer(allocator, track_type8::vqvf, k_num_transforms);

	// With the wrap policy, half a frame before the end interpolates the last stored sample and the first sample
	context.seek(k_duration - (0.5F / k_sample_rate), sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 1.5F, k_error_budget));

	// With the clamp policy, the clip is one frame shorter and the time clamps to the last stored sample
	context.set_looping_policy(sample_looping_policy::clamp);
	CHECK(context.get_looping_policy() == sample_looping_policy::clamp);

	context.seek(k_duration - (0.5F / k_sample_rate), sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 3.0F, k_error_budget));

	context.seek(k_duration, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(get_translation_x(writer.read_vqv(0)), 3.0F, k_error_budget));

	allocator.deallocate(tracks, tracks->get_size());
}

TEST_CASE("loop optimization wraps a looping scalar clip", "[compression][scalar][loop]")
{
	ansi_allocator allocator;

	track_array_float1f raw_tracks = make_scalar_clip(allocator, k_num_samples, k_loop_offset);
	REQUIRE(raw_tracks.get_num_samples_per_track() == k_num_samples);
	const float raw_duration = raw_tracks.get_duration();
	REQUIRE(rtm::scalar_near_equal(raw_duration, k_duration, 1.0E-6F));

	compressed_tracks* tracks = compress_clip(allocator, raw_tracks, true);

	// The last sample matches the first sample: it is removed and the wrap policy applies
	CHECK(tracks->get_looping_policy() == sample_looping_policy::wrap);
	CHECK(tracks->get_num_samples_per_track() == k_num_samples - 1);

	// The removed sample repeats the first sample: the playback duration stays the same
	CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::as_compressed), raw_duration, 1.0E-6F));
	CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::clamp), raw_duration - (1.0F / k_sample_rate), 1.0E-6F));

	decompression_context<debug_scalar_decompression_settings> context;
	REQUIRE(context.initialize(*tracks));
	CHECK(context.get_looping_policy() == sample_looping_policy::wrap);

	debug_track_writer writer(allocator, track_type8::float1f, 1);

	// The start of the clip holds the first sample
	context.seek(0.0F, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(writer.read_float1(0), 0.0F, k_error_budget));

	// The end of the clip wraps to the first sample and stays within the precision budget of the raw last sample
	context.seek(k_duration, sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(writer.read_float1(0), 0.0F, k_error_budget));
	CHECK(std::fabs(writer.read_float1(0) - k_loop_offset) <= k_error_budget);

	// Half a frame before the end, the clip interpolates the last stored sample and the first sample.
	// A clip that clamps or that plays one frame less would return the last stored sample (3.0).
	context.seek(k_duration - (0.5F / k_sample_rate), sample_rounding_policy::none);
	context.decompress_tracks(writer);
	CHECK(rtm::scalar_near_equal(writer.read_float1(0), 1.5F, k_error_budget));

	allocator.deallocate(tracks, tracks->get_size());
}

TEST_CASE("loop optimization keeps the scalar contract when disabled or not looping", "[compression][scalar][loop]")
{
	ansi_allocator allocator;

	// The optimization is off: every sample is retained and the clamp policy applies
	{
		track_array_float1f raw_tracks = make_scalar_clip(allocator, k_num_samples, k_loop_offset);
		compressed_tracks* tracks = compress_clip(allocator, raw_tracks, false);

		CHECK(tracks->get_looping_policy() == sample_looping_policy::clamp);
		CHECK(tracks->get_num_samples_per_track() == k_num_samples);
		CHECK(rtm::scalar_near_equal(tracks->get_duration(sample_looping_policy::as_compressed), k_duration, 1.0E-6F));

		decompression_context<debug_scalar_decompression_settings> context;
		REQUIRE(context.initialize(*tracks));

		debug_track_writer writer(allocator, track_type8::float1f, 1);

		// The end of the clip holds the true last sample
		context.seek(k_duration, sample_rounding_policy::none);
		context.decompress_tracks(writer);
		CHECK(rtm::scalar_near_equal(writer.read_float1(0), k_loop_offset, k_error_budget));

		allocator.deallocate(tracks, tracks->get_size());
	}

	// The last sample differs from the first sample by more than the precision: the clip does not loop
	{
		track_array_float1f raw_tracks = make_scalar_clip(allocator, k_num_samples, k_no_loop_offset);
		compressed_tracks* tracks = compress_clip(allocator, raw_tracks, true);

		CHECK(tracks->get_looping_policy() == sample_looping_policy::clamp);
		CHECK(tracks->get_num_samples_per_track() == k_num_samples);

		decompression_context<debug_scalar_decompression_settings> context;
		REQUIRE(context.initialize(*tracks));

		debug_track_writer writer(allocator, track_type8::float1f, 1);

		// The end of the clip holds the true last sample: the difference with the first sample is retained
		context.seek(k_duration, sample_rounding_policy::none);
		context.decompress_tracks(writer);
		CHECK(rtm::scalar_near_equal(writer.read_float1(0), k_no_loop_offset, k_error_budget));

		allocator.deallocate(tracks, tracks->get_size());
	}

	// A single sample cannot wrap
	{
		track_array_float1f raw_tracks = make_scalar_clip(allocator, 1, 2.5F);
		compressed_tracks* tracks = compress_clip(allocator, raw_tracks, true);

		CHECK(tracks->get_looping_policy() == sample_looping_policy::clamp);
		CHECK(tracks->get_num_samples_per_track() == 1);
		CHECK(tracks->get_finite_duration(sample_looping_policy::as_compressed) == 0.0F);

		decompression_context<debug_scalar_decompression_settings> context;
		REQUIRE(context.initialize(*tracks));

		debug_track_writer writer(allocator, track_type8::float1f, 1);

		context.seek(0.0F, sample_rounding_policy::none);
		context.decompress_tracks(writer);
		CHECK(rtm::scalar_near_equal(writer.read_float1(0), 2.5F, k_error_budget));

		allocator.deallocate(tracks, tracks->get_size());
	}
}
