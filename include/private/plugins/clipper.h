/*
 * Copyright (C) 2023 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2023 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-clipper
 * Created on: 01 дек 2023 г.
 *
 * lsp-plugins-clipper is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * lsp-plugins-clipper is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lsp-plugins-clipper. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef PRIVATE_PLUGINS_CLIPPER_H_
#define PRIVATE_PLUGINS_CLIPPER_H_

#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/dsp-units/dynamics/SimpleAutoGain.h>
#include <lsp-plug.in/dsp-units/meters/LoudnessMeter.h>
#include <lsp-plug.in/dsp-units/misc/sigmoid.h>
#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/dsp-units/util/Dither.h>
#include <lsp-plug.in/dsp-units/util/MeterGraph.h>
#include <lsp-plug.in/dsp-units/util/Sidechain.h>
#include <lsp-plug.in/plug-fw/core/IDBuffer.h>
#include <lsp-plug.in/plug-fw/plug.h>
#include <private/meta/clipper.h>

namespace lsp
{
    namespace plugins
    {
        /**
         * Clipper plugin
         */
        class clipper: public plug::Module
        {
            protected:
                enum clipper_flags_t
                {
                    CF_BOOSTING         = 1 << 0,           // Enable gain boosting
                    CF_LUFS_LIMITER     = 1 << 1,           // Enable LUFS limiter
                    CF_CLIP_ENABLED     = 1 << 2,           // Output clipper enabled
                    CF_ODP_ENABLED      = 1 << 3,           // Overdrive protection enabled
                    CF_DC_COMPENSATE    = 1 << 4,           // Compensate DC offset
                    CF_SYNC_ODP         = 1 << 5,           // Sync overdrive protection curve
                    CF_SYNC_CLIP        = 1 << 6,           // Sync sigmoid clipping curve

                    CF_SYNC_ALL         = CF_SYNC_ODP | CF_SYNC_CLIP
                };

                enum channel_flags_t
                {
                    CH_IN_GRAPH         = 1 << 0,           // Input graph visibility
                    CH_OUT_GRAPH        = 1 << 1,           // Output graph visibility
                    CH_RED_GRAPH        = 1 << 2            // Gain graph visibility
                };

                typedef struct compressor_t
                {
                    float       x0, x1, x2;
                    float       t;
                    float       a, b, c;
                } compressor_t;

                // Overdrive protection module
                typedef struct odp_params_t
                {
                    float               fThreshold;         // Threshold
                    float               fKnee;              // Knee

                    plug::IPort        *pOn;                // Enable overdrive protection
                    plug::IPort        *pThreshold;         // Threshold
                    plug::IPort        *pKnee;              // Knee
                    plug::IPort        *pReactivity;        // Reactivity
                    plug::IPort        *pCurveMesh;         // Curve chart mesh
                } odp_params_t;

                typedef struct clip_params_t
                {
                    dspu::sigmoid::function_t   pFunc;      // Sigmoid function
                    float               fThreshold;         // Threshold
                    float               fDCOffset;          // DC offset
                    float               fPumping;           // Pumping
                    float               fScaling;           // Sigmoid scaling
                    float               fKnee;              // Knee

                    plug::IPort        *pOn;                // Enable sigmoid function
                    plug::IPort        *pFunction;          // Sigmoid function
                    plug::IPort        *pThreshold;         // Sigmoid threshold
                    plug::IPort        *pDCOffset;          // DC offset
                    plug::IPort        *pDCCompensate;      // DC compensate
                    plug::IPort        *pPumping;           // Sigmoid pumping
                    plug::IPort        *pCurveMesh;         // Curve chart mesh
                } clip_params_t;

                typedef struct lufs_limiter_t
                {
                    dspu::LoudnessMeter sMeter;             // Input LUFS meter
                    dspu::SimpleAutoGain sGain;             // Input LUFS limiter
                    float               fIn;                // Input level
                    float               fRed;               // Gain reduction

                    plug::IPort        *pOn;                // Enable LUFS limiter
                    plug::IPort        *pIn;                // LUFS input level
                    plug::IPort        *pRed;               // LUFS gain reduction
                    plug::IPort        *pThreshold;         // LUFS limiter threshold
                } lufs_limiter_t;

                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Bypass        sBypass;            // Bypass
                    dspu::Delay         sDryDelay;          // Delay for the dry signal
                    dspu::Delay         sScDelay;           // Sidechain compensation delay
                    dspu::Sidechain     sSc;                // Sidechain
                    dspu::Dither        sDither;            // Dither
                    dspu::MeterGraph    sInGraph;           // Input meter graph
                    dspu::MeterGraph    sOutGraph;          // Output meter graph
                    dspu::MeterGraph    sRedGraph;          // Reduction meter graph

                    // Channel flags
                    uint32_t            nFlags;             // Channel flags

                    // Meter values
                    float               fIn;                // Input level meter
                    float               fOut;               // Output level meter
                    float               fRed;               // Reduction level meter

                    float               fOdpIn;             // Overdrive protection input level
                    float               fOdpOut;            // Overdrive protection out level
                    float               fOdpRed;            // Overdrive protection reduction level

                    float               fClipIn[2];         // Clipping input level measured
                    float               fClipOut[2];        // Clipping output level measured
                    float               fClipRed;           // Clipping reduction level measured

                    // Buffers
                    float              *vIn;                // Input buffer
                    float              *vOut;               // Output buffer
                    float              *vInMeter;           // Input data metering
                    float              *vRedMeter;          // Reduction metering
                    float              *vData;              // Data buffer
                    float              *vSc;                // Sidechain buffer

                    // Input ports
                    plug::IPort        *pDataIn;            // Input port
                    plug::IPort        *pDataOut;           // Output port

                    // Metering
                    plug::IPort        *pInVisible;         // Input visibility
                    plug::IPort        *pOutVisible;        // Output visibility
                    plug::IPort        *pRedVisible;        // Reduction visibility

                    plug::IPort        *pIn;                // Input level meter
                    plug::IPort        *pOut;               // Output level meter
                    plug::IPort        *pRed;               // Reduction level meter

                    plug::IPort        *pOdpIn;             // ODP input level meter
                    plug::IPort        *pOdpOut;            // ODP output level meter
                    plug::IPort        *pOdpRed;            // ODP reduction level meter

                    plug::IPort        *pClipIn[2];         // Clipping input level meter
                    plug::IPort        *pClipOut[2];        // Clipping output level meter
                    plug::IPort        *pClipRed;           // Clipping reduction level meter

                    plug::IPort        *pTimeMesh;          // Input, output and gain reduction graph mesh
                } channel_t;

            protected:
                static dspu::sigmoid::function_t    vSigmoidFunctions[];

            protected:
                size_t              nChannels;          // Number of channels
                channel_t          *vChannels;          // Delay channels

                dspu::LoudnessMeter sInMeter;           // Input LUFS meter
                dspu::LoudnessMeter sOutMeter;          // Output LUFS meter
                compressor_t        sComp;              // Simple compressor
                odp_params_t        sOdp;               // Overdrive protection params
                clip_params_t       sClip;              // Clipping parameters
                lufs_limiter_t      sLufs;              // LUFS limiter

                float               fInGain;            // Input gain
                float               fOutGain;           // Output gain
                float               fInLufs;            // Output LUFS
                float               fOutLufs;           // Output LUFS
                float               fThresh;            // Threshold
                float               fStereoLink;        // Stereo link
                uint32_t            nFlags;             // Global flags

                float              *vBuffer;            // Temporary buffer
                float              *vOdp;               // Overdrive protection curve input gain values
                float              *vLinSigmoid;        // Linear scale for sigmoid
                float              *vLogSigmoid;        // Logarithmic scale for sigmoid
                float              *vTime;              // Time graph
                core::IDBuffer     *pIDisplay;          // Inline display buffer

                plug::IPort        *pBypass;            // Bypass
                plug::IPort        *pGainIn;            // Input gain
                plug::IPort        *pGainOut;           // Output gain
                plug::IPort        *pLufsIn;            // Input LUFS meter
                plug::IPort        *pLufsOut;           // Output LUFS meter
                plug::IPort        *pThresh;            // Threshold
                plug::IPort        *pBoosting;          // Boosting mode
                plug::IPort        *pStereoLink;        // Stereo linking for output clipper
                plug::IPort        *pDithering;         // Dithering mode

                uint8_t            *pData;              // Allocated data

            protected:
                static size_t           decode_dithering(size_t mode);

                static bool             update_odp_params(odp_params_t *params);
                static bool             update_clip_params(clip_params_t *params);

                static void             calc_odp_compressor(compressor_t *c, const odp_params_t *params);
                static inline float     odp_curve(const compressor_t *c, float x);
                static inline float     odp_gain(const compressor_t *c, float x);
                static void             odp_curve(float *dst, const float *x, const compressor_t *c, size_t count);
                static void             odp_gain(float *dst, const float *x, const compressor_t *c, size_t count);
                static void             odp_link(float *dst, const float *src, float link, size_t count);

                static float            clip_curve(const clip_params_t *p, float x);
                static void             clip_curve(float *dst, const float *x, const clip_params_t *p, size_t count);

            protected:
                void                    do_destroy();
                void                    bind_input_buffers();
                void                    process_odp_channel(channel_t *c, size_t samples);
                void                    process_clip_channel(channel_t *c, size_t samples);
                void                    meter_channel(channel_t *c, size_t samples);
                void                    process_clipper(size_t samples);
                void                    output_signal(size_t samples);
                void                    advance_buffers(size_t samples);
                void                    output_meters();
                void                    output_mesh_curves(size_t samples);

            public:
                explicit clipper(const meta::plugin_t *meta);
                clipper (const clipper &) = delete;
                clipper (clipper &&) = delete;
                virtual ~clipper() override;

                clipper & operator = (const clipper &) = delete;
                clipper & operator = (clipper &&) = delete;

                virtual void            init(plug::IWrapper *wrapper, plug::IPort **ports) override;
                virtual void            destroy() override;

            public:
                virtual void            update_sample_rate(long sr) override;
                virtual void            update_settings() override;
                virtual void            process(size_t samples) override;
                virtual void            ui_activated() override;
                virtual bool            inline_display(plug::ICanvas *cv, size_t width, size_t height) override;
                virtual void            dump(dspu::IStateDumper *v) const override;
        };

    } /* namespace plugins */
} /* namespace lsp */


#endif /* PRIVATE_PLUGINS_CLIPPER_H_ */

