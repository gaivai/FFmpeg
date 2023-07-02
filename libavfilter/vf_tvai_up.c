/*
 * Copyright (c) 2022 Topaz Labs LLC
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Topaz Video AI Upscale filter
 *
 * @see https://www.topazlabs.com/topaz-video-ai
 */

#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/avutil.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"
#include "tvai.h"
#include "tvai_common.h"

typedef struct TVAIUpContext {
    const AVClass *class;
    char *model;
    int device, scale, extraThreads;
    int canDownloadModels;
    int estimateFrameCount, count, estimating, w, h, canKeepColor;
    double vram;
    double preBlur, noise, details, halo, blur, compression;
    double prenoise, grain, grainSize, blend;
    void* pFrameProcessor;
    AVFrame* previousFrame;
} TVAIUpContext;

#define OFFSET(x) offsetof(TVAIUpContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption tvai_up_options[] = {
    { "model", "Model short name", OFFSET(model), AV_OPT_TYPE_STRING, {.str="amq-13"}, .flags = FLAGS },
    { "scale",  "Output scale",  OFFSET(scale),  AV_OPT_TYPE_INT, {.i64=1}, 0, 4, FLAGS, "scale" },
    { "w",  "Estimate scale based on output width",  OFFSET(w),  AV_OPT_TYPE_INT, {.i64=0}, 0, 100000, FLAGS, "w" },
    { "h",  "Estimate scale based on output height",  OFFSET(h),  AV_OPT_TYPE_INT, {.i64=0}, 0, 100000, FLAGS, "h" },
    { "device",  "Device index (Auto: -2, CPU: -1, GPU0: 0, ...)",  OFFSET(device),  AV_OPT_TYPE_INT, {.i64=-2}, -2, 8, FLAGS, "device" },
    { "instances",  "Number of extra model instances to use on device",  OFFSET(extraThreads),  AV_OPT_TYPE_INT, {.i64=0}, 0, 3, FLAGS, "instances" },
    { "download",  "Enable model downloading",  OFFSET(canDownloadModels),  AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS, "canDownloadModels" },
    { "vram", "Max memory usage", OFFSET(vram), AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0.1, 1, .flags = FLAGS, "vram"},
    { "estimate",  "Number of frames for auto parameter estimation, 0 to disable auto parameter estimation",  OFFSET(estimateFrameCount),  AV_OPT_TYPE_INT, {.i64=0}, 0, 1000000, FLAGS, "estimateParamNthFrame" },
    { "preblur",  "Adjusts both the antialiasing and deblurring strength relative to the amount of aliasing and blurring in the input video. \nNegative values are better if the input video has aliasing artifacts such as moire patterns or staircasing. Positive values are better if the input video has more lens blurring than aliasing artifacts. ",  OFFSET(preBlur),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1.0, 1.0, FLAGS, "preblur" },
    { "noise",  "Removes ISO noise from the input video. Higher values remove more noise but may also remove fine details. \nNote that this value is relative to the amount of noise found in the input video - higher values on videos with low amounts of ISO noise may introduce more artifacts.",  OFFSET(noise),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1.0, 1.0, FLAGS, "noise" },
    { "details",  "Used to recover fine texture and detail lost due to in-camera noise suppression. \nThis value is relative to the amount of noise suppression in the camera used for the input video, and higher values may introduce artifacts if the input video has little to no in-camera noise suppression.",  OFFSET(details),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1.0, 1.0, FLAGS, "details" },
    { "halo",  "Increase this if the input video has halo or ring artifacts around strong edges caused by oversharpening. \nThis value is relative to the amount of haloing artifacts in the input video, and has a \"sweet spot\". Values that are too high for the input video may cause additional artifacts to appear.",  OFFSET(halo),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1.0, 1.0, FLAGS, "halo" },
    { "blur",  "Additional sharpening of the video. Use this if the input video looks too soft. \nThe value set should be relative to the amount of softness in the input video - if the input video is already sharp, higher values will introduce more artifacts.",  OFFSET(blur),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1.0, 1.0, FLAGS, "blur" },
    { "compression",  "Reduces compression artifacts from codec encoding, such as blockiness or mosquito noise. Higher values are best for low bitrate videos.\nNote that the value should be relative to the amount of compression artifacts in the input video - higher values on a video with few compression artifacts will introduce more artifacts into the output.",  OFFSET(compression),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1.0, 1.0, FLAGS, "compression" },
    { "prenoise",  "The amount of noise to add to the input before processing",  OFFSET(prenoise),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0.0, 0.1, FLAGS, "prenoise" },
    { "grain",  "The amount of grain to add to the output",  OFFSET(grain),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0.0, 0.1, FLAGS, "grain" },
    { "gsize",  "The size of grain to be added",  OFFSET(grainSize),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0.0, 5.0, FLAGS, "gsize" },
    { "kcolor",  "Run extra color correction if required by model",  OFFSET(canKeepColor),  AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS, "kcolor" },
    { "blend",  "The amount of input to be blended with output",  OFFSET(blend),  AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0.0, 1.0, FLAGS, "blend" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(tvai_up);

static av_cold int init(AVFilterContext *ctx) {
  TVAIUpContext *tvai = ctx->priv;
  av_log(ctx, AV_LOG_VERBOSE, "Here init with params: %s %d %d %lf %lf %lf %lf %lf %lf\n", tvai->model, tvai->scale, tvai->device,
        tvai->preBlur, tvai->noise, tvai->details, tvai->halo, tvai->blur, tvai->compression);
  tvai->previousFrame = NULL;
  tvai->count = 0;
  return 0;
}

static int config_props(AVFilterLink *outlink) {
    AVFilterContext *ctx = outlink->src;
    TVAIUpContext *tvai = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    float parameter_values[12] = {tvai->preBlur, tvai->noise, tvai->details, tvai->halo, tvai->blur, tvai->compression, 0, tvai->prenoise, tvai->grain, tvai->grainSize, tvai->canKeepColor, tvai->blend};
    VideoProcessorInfo info;
    int scale = tvai->scale;
    double sar = av_q2d(inlink->sample_aspect_ratio) > 0 ? av_q2d(inlink->sample_aspect_ratio) : 1;
    if(scale == 0) {
      //float x = tvai->w/(sar*inlink->w), y = tvai->h*1.0f/inlink->h;
      float x = tvai->w/inlink->w, y = tvai->h*1.0f/inlink->h;
      float v = x > y ? x : y;
      scale = (v > 2.4) ? 4 : (v > 1.2 ? 2 : 1);
      av_log(ctx, AV_LOG_VERBOSE, "SAR: %lf scale: %d x: %f y: %f v: %f\n", sar, scale, x, y, v);
    }
    info.frameCount = tvai->estimateFrameCount;
    av_log(ctx, AV_LOG_VERBOSE, "Here init with perf options: model: %s scale: %d device: %d vram: %lf threads: %d downloads: %d\n", tvai->model, tvai->scale, tvai->device,tvai->vram, tvai->extraThreads, tvai->canDownloadModels);
    if(ff_tvai_verifyAndSetInfo(&info, inlink, outlink, tvai->estimateFrameCount > 0, tvai->model, ModelTypeUpscaling, tvai->device, tvai->extraThreads,    tvai->vram, scale, tvai->canDownloadModels, parameter_values, 12, ctx)) {
      return AVERROR(EINVAL);
    }
    tvai->pFrameProcessor = tvai_create(&info);
    tvai->previousFrame = NULL;
    return tvai->pFrameProcessor == NULL ? AVERROR(EINVAL) : 0;
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_RGB48,
    AV_PIX_FMT_NONE
};

static int filter_frame(AVFilterLink *inlink, AVFrame *in) {
    AVFilterContext *ctx = inlink->dst;
    TVAIUpContext *tvai = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    int ret = 0;
    if(ff_tvai_process(tvai->pFrameProcessor, in, 0)) {
        av_log(NULL, AV_LOG_ERROR, "The processing has failed\n");
        av_frame_free(&in);
        return AVERROR(ENOSYS);
    }
    if(tvai->previousFrame)
        av_frame_free(&tvai->previousFrame);
    tvai->previousFrame = in;
    ret = ff_tvai_add_output(tvai->pFrameProcessor, outlink, in, 0);
    return ret;
}

static int request_frame(AVFilterLink *outlink) {
    AVFilterContext *ctx = outlink->src;
    TVAIUpContext *tvai = ctx->priv;
    int ret = ff_request_frame(ctx->inputs[0]);
    if (ret == AVERROR_EOF) {
        int r = ff_tvai_postflight(outlink, tvai->pFrameProcessor, tvai->previousFrame);
        if(r)
            return r;
    }
    return ret;
}

static av_cold void uninit(AVFilterContext *ctx) {
    TVAIUpContext *tvai = ctx->priv;
    av_log(ctx, AV_LOG_DEBUG, "Uninit called for %s %d\n", tvai->model, tvai->pFrameProcessor == NULL);
    // if(tvai->pFrameProcessor)
    //     tvai_destroy(tvai->pFrameProcessor);
}

static const AVFilterPad tvai_up_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad tvai_up_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .config_props = config_props,
        .request_frame = request_frame,
    },
};

const AVFilter ff_vf_tvai_up = {
    .name          = "tvai_up",
    .description   = NULL_IF_CONFIG_SMALL("Apply Topaz Video AI upscale models, parameters will only be applied to appropriate models"),
    .priv_size     = sizeof(TVAIUpContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(tvai_up_inputs),
    FILTER_OUTPUTS(tvai_up_outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
    .priv_class    = &tvai_up_class,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
};
