/*
 * Copyright (C) 2025 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2025 Vladimir Sadovnikov <sadko4u@gmail.com>
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

#include <lsp-plug.in/common/alloc.h>
#include <lsp-plug.in/common/debug.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/plug-fw/meta/func.h>
#include <lsp-plug.in/shared/debug.h>
#include <lsp-plug.in/shared/id_colors.h>

#include <private/plugins/clipper.h>

namespace lsp
{
    namespace plugins
    {
        /* The size of temporary buffer for audio processing */
        static constexpr size_t BUFFER_SIZE     = 0x400;

        //---------------------------------------------------------------------
        // Plugin factory
        static const meta::plugin_t *plugins[] =
        {
            &meta::clipper_mono,
            &meta::clipper_stereo
        };

        static plug::Module *plugin_factory(const meta::plugin_t *meta)
        {
            return new clipper(meta);
        }

        static plug::Factory factory(plugin_factory, plugins, 2);

        //---------------------------------------------------------------------
        // Implementation
        dspu::sigmoid::function_t clipper::vSigmoidFunctions[] =
        {
            dspu::sigmoid::hard_clip,
            dspu::sigmoid::quadratic,
            dspu::sigmoid::sine,
            dspu::sigmoid::logistic,
            dspu::sigmoid::arctangent,
            dspu::sigmoid::hyperbolic_tangent,
            dspu::sigmoid::guidermannian,
            dspu::sigmoid::error,
            dspu::sigmoid::smoothstep,
            dspu::sigmoid::smootherstep,
            dspu::sigmoid::circle
        };


        clipper::clipper(const meta::plugin_t *meta):
            Module(meta)
        {
            // Compute the number of audio channels by the number of inputs
            nChannels       = 0;
            for (const meta::port_t *p = meta->ports; p->id != NULL; ++p)
                if (meta::is_audio_in_port(p))
                    ++nChannels;

            // Initialize other parameters
            vChannels       = NULL;

            sComp.x0                = 0.0f;
            sComp.x1                = 0.0f;
            sComp.x2                = 0.0f;
            sComp.t                 = 0.0f;
            sComp.a                 = 0.0f;
            sComp.b                 = 0.0f;
            sComp.c                 = 0.0f;

            sOdp.fThreshold         = 0.0f;
            sOdp.fKnee              = 0.0f;

            sOdp.pThreshold         = NULL;
            sOdp.pKnee              = NULL;
            sOdp.pReactivity        = NULL;
            sOdp.pCurveMesh         = NULL;

            sClip.pFunc             = NULL;
            sClip.fThreshold        = 0.0f;
            sClip.fDCOffset         = 0.0f;
            sClip.fPumping          = 1.0f;
            sClip.fScaling          = 0.0f;
            sClip.fKnee             = 0.0f;

            sClip.pOn               = NULL;
            sClip.pFunction         = NULL;
            sClip.pThreshold        = NULL;
            sClip.pDCOffset         = NULL;
            sClip.pDCCompensate     = NULL;
            sClip.pPumping          = NULL;
            sClip.pCurveMesh        = NULL;

            sLufs.fIn               = GAIN_AMP_M_INF_DB;
            sLufs.fRed              = GAIN_AMP_0_DB;
            sLufs.pOn               = NULL;
            sLufs.pIn               = NULL;
            sLufs.pRed              = NULL;
            sLufs.pThreshold        = NULL;

            fInGain                 = GAIN_AMP_0_DB;
            fOutGain                = GAIN_AMP_0_DB;
            fInLufs                 = GAIN_AMP_M_INF_DB;
            fOutLufs                = GAIN_AMP_M_INF_DB;
            fThresh                 = GAIN_AMP_0_DB;
            fStereoLink             = 0.0f;
            nFlags                  = CF_SYNC_ALL;

            vBuffer                 = NULL;
            vOdp                    = NULL;
            vLinSigmoid             = NULL;
            vLogSigmoid             = NULL;
            vTime                   = NULL;
            vWaveformTime           = NULL;
            pIDisplay               = NULL;

            pBypass                 = NULL;
            pGainIn                 = NULL;
            pGainOut                = NULL;
            pLufsIn                 = NULL;
            pLufsOut                = NULL;
            pThresh                 = NULL;
            pBoosting               = NULL;
            pStereoLink             = NULL;
            pDithering              = NULL;
            pTimeMesh               = NULL;
            pWaveformMesh           = NULL;

            pData                   = NULL;
        }

        clipper::~clipper()
        {
            do_destroy();
        }

        void clipper::destroy()
        {
            Module::destroy();
            do_destroy();
        }

        void clipper::do_destroy()
        {
            // Destroy channels
            if (vChannels != NULL)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];

                    c->sBypass.destroy();
                    c->sDryDelay.destroy();
                    c->sScDelay.destroy();
                    c->sSc.destroy();
                    c->sDither.destroy();
                    c->sInGraph.destroy();
                    c->sOutGraph.destroy();
                    c->sRedGraph.destroy();
                    c->sWaveformGraph.destroy();
                }
                vChannels   = NULL;
            }

            // Destroy inline display
            if (pIDisplay != NULL)
            {
                pIDisplay->destroy();
                pIDisplay   = NULL;
            }

            // Free previously allocated data block
            free_aligned(pData);
        }

        void clipper::init(plug::IWrapper *wrapper, plug::IPort **ports)
        {
            // Call parent class for initialization
            Module::init(wrapper, ports);

            // Call parent class for initialization
            Module::init(wrapper, ports);

            // Estimate the number of bytes to allocate
            size_t szof_channels    = align_size(sizeof(channel_t) * nChannels, OPTIMAL_ALIGN);
            size_t szof_buffer      = align_size(sizeof(float) * BUFFER_SIZE, OPTIMAL_ALIGN);
            size_t szof_curve_buffer= align_size(sizeof(float) * meta::clipper::CURVE_MESH_POINTS, OPTIMAL_ALIGN);
            size_t szof_time_buffer = align_size(sizeof(float) * meta::clipper::TIME_MESH_POINTS, OPTIMAL_ALIGN);
            size_t to_alloc         =
                szof_channels +
                szof_buffer +           // vBuffer
                szof_curve_buffer +     // vOdp
                szof_curve_buffer +     // vLinSigmoid
                szof_curve_buffer +     // vLogSigmoid
                szof_time_buffer +      // vTime
                szof_time_buffer +      // vWaveformTime
                nChannels * (
                    szof_buffer +       // vInMeter
                    szof_buffer +       // vRedMeter
                    szof_buffer +       // vData
                    szof_buffer         // vSc
                );

            // Initialize analyzer
            sLufs.sMeter.construct();
            sLufs.sGain.construct();

            sInMeter.construct();
            sInMeter.init(nChannels, meta::clipper::LUFS_MEASUREMENT_PERIOD);
            sInMeter.set_period(meta::clipper::LUFS_MEASUREMENT_PERIOD);
            sInMeter.set_weighting(dspu::bs::WEIGHT_K);
            if (nChannels > 1)
            {
                sInMeter.set_designation(0, dspu::bs::CHANNEL_LEFT);
                sInMeter.set_designation(1, dspu::bs::CHANNEL_RIGHT);
            }
            else
                sInMeter.set_designation(0, dspu::bs::CHANNEL_CENTER);

            sOutMeter.construct();
            sOutMeter.init(nChannels, meta::clipper::LUFS_MEASUREMENT_PERIOD);
            sOutMeter.set_period(meta::clipper::LUFS_MEASUREMENT_PERIOD);
            sOutMeter.set_weighting(dspu::bs::WEIGHT_K);
            if (nChannels > 1)
            {
                sOutMeter.set_designation(0, dspu::bs::CHANNEL_LEFT);
                sOutMeter.set_designation(1, dspu::bs::CHANNEL_RIGHT);
            }
            else
                sOutMeter.set_designation(0, dspu::bs::CHANNEL_CENTER);

            sLufs.sMeter.init(nChannels, meta::clipper::LUFS_MEASUREMENT_PERIOD);
            sLufs.sMeter.set_period(meta::clipper::LUFS_MEASUREMENT_PERIOD);
            sLufs.sMeter.set_weighting(dspu::bs::WEIGHT_K);
            sLufs.sGain.init();
            sLufs.sGain.set_speed(meta::clipper::LUFS_LIMITER_REACT, meta::clipper::LUFS_LIMITER_REACT);
            if (nChannels > 1)
            {
                sLufs.sMeter.set_designation(0, dspu::bs::CHANNEL_LEFT);
                sLufs.sMeter.set_designation(1, dspu::bs::CHANNEL_RIGHT);
            }
            else
                sLufs.sMeter.set_designation(0, dspu::bs::CHANNEL_CENTER);

            // Allocate memory-aligned data
            uint8_t *ptr            = alloc_aligned<uint8_t>(pData, to_alloc, OPTIMAL_ALIGN);
            if (ptr == NULL)
                return;
            lsp_guard_assert( const uint8_t *tail = &ptr[to_alloc]; );

            // Initialize pointers to channels and temporary buffer
            vChannels               = advance_ptr_bytes<channel_t>(ptr, szof_channels);
            vBuffer                 = advance_ptr_bytes<float>(ptr, szof_buffer);
            vOdp                    = advance_ptr_bytes<float>(ptr, szof_curve_buffer);
            vLinSigmoid             = advance_ptr_bytes<float>(ptr, szof_curve_buffer);
            vLogSigmoid             = advance_ptr_bytes<float>(ptr, szof_curve_buffer);
            vTime                   = advance_ptr_bytes<float>(ptr, szof_time_buffer);
            vWaveformTime           = advance_ptr_bytes<float>(ptr, szof_time_buffer);

            for (size_t i=0; i < nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Construct in-place DSP processors
                c->sBypass.construct();
                c->sDryDelay.construct();
                c->sScDelay.construct();
                c->sSc.construct();
                c->sDither.construct();

                c->sInGraph.construct();
                c->sOutGraph.construct();
                c->sRedGraph.construct();
                c->sWaveformGraph.construct();
                c->sInGraph.set_method(dspu::MM_ABS_MAXIMUM);
                c->sOutGraph.set_method(dspu::MM_ABS_MAXIMUM);
                c->sRedGraph.set_method(dspu::MM_ABS_MINIMUM);
                c->sWaveformGraph.set_method(dspu::MM_VAR_MINMAX);

                c->sDither.init();

                c->nFlags               = 0;

                c->fIn                  = GAIN_AMP_M_INF_DB;
                c->fOut                 = GAIN_AMP_M_INF_DB;
                c->fRed                 = GAIN_AMP_M_INF_DB;

                c->fOdpIn               = GAIN_AMP_M_INF_DB;
                c->fOdpOut              = GAIN_AMP_M_INF_DB;
                c->fOdpRed              = GAIN_AMP_M_INF_DB;

                c->fClipIn[0]           = GAIN_AMP_M_INF_DB;
                c->fClipIn[1]           = GAIN_AMP_M_INF_DB;
                c->fClipOut[0]          = GAIN_AMP_M_INF_DB;
                c->fClipOut[1]          = GAIN_AMP_M_INF_DB;
                c->fClipRed             = GAIN_AMP_M_INF_DB;

                c->vIn                  = NULL;
                c->vOut                 = NULL;
                c->vInMeter             = advance_ptr_bytes<float>(ptr, szof_buffer);
                c->vRedMeter            = advance_ptr_bytes<float>(ptr, szof_buffer);
                c->vData                = advance_ptr_bytes<float>(ptr, szof_buffer);
                c->vSc                  = advance_ptr_bytes<float>(ptr, szof_buffer);

                // Initialize ports
                c->pDataIn              = NULL;
                c->pDataOut             = NULL;

                c->pInVisible           = NULL;
                c->pOutVisible          = NULL;
                c->pRedVisible          = NULL;

                c->pIn                  = NULL;
                c->pOut                 = NULL;
                c->pRed                 = NULL;

                c->pOdpIn               = NULL;
                c->pOdpOut              = NULL;
                c->pOdpRed              = NULL;

                c->pClipIn[0]           = NULL;
                c->pClipIn[1]           = NULL;
                c->pClipOut[0]          = NULL;
                c->pClipOut[1]          = NULL;
                c->pClipRed             = NULL;
            }

            lsp_assert( ptr <= tail );

            // Bind ports
            lsp_trace("Binding input ports");
            size_t port_id      = 0;

            // Bind input audio ports
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i].pDataIn);

            // Bind output audio ports
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i].pDataOut);

            // Bind common ports
            lsp_trace("Binding common ports");
            BIND_PORT(pBypass);
            BIND_PORT(pGainIn);
            BIND_PORT(pGainOut);
            BIND_PORT(sLufs.pOn);
            BIND_PORT(sLufs.pThreshold);
            BIND_PORT(sLufs.pIn);
            BIND_PORT(sLufs.pRed);
            BIND_PORT(pLufsIn);
            BIND_PORT(pLufsOut);
            BIND_PORT(pThresh);
            BIND_PORT(pBoosting);
            BIND_PORT(pDithering);
            SKIP_PORT("Linear/Logarithmic view"); // Skip clipper linear/logarithmic graph view

            // Bind clipper ports
            lsp_trace("Binding clipper ports");
            BIND_PORT(sOdp.pOn);
            BIND_PORT(sOdp.pThreshold);
            BIND_PORT(sOdp.pKnee);
            BIND_PORT(sOdp.pReactivity);
            BIND_PORT(sOdp.pCurveMesh);
            BIND_PORT(sClip.pOn);
            BIND_PORT(sClip.pFunction);
            BIND_PORT(sClip.pThreshold);
            BIND_PORT(sClip.pDCOffset);
            BIND_PORT(sClip.pDCCompensate);
            BIND_PORT(sClip.pPumping);
            BIND_PORT(sClip.pCurveMesh);
            SKIP_PORT("Clipper graph view");
            BIND_PORT(pTimeMesh);
            BIND_PORT(pWaveformMesh);
            if (nChannels > 1)
                BIND_PORT(pStereoLink);

            lsp_trace("Skipping graph visibility ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                BIND_PORT(c->pInVisible);
                BIND_PORT(c->pOutVisible);
                BIND_PORT(c->pRedVisible);
            }

            // Bind channel metering
            lsp_trace("Binding channel metering ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                BIND_PORT(c->pIn);
                BIND_PORT(c->pOut);
                BIND_PORT(c->pRed);

                BIND_PORT(c->pOdpIn);
                BIND_PORT(c->pOdpOut);
                BIND_PORT(c->pOdpRed);

                BIND_PORT(c->pClipIn[0]);
                BIND_PORT(c->pClipOut[0]);
                BIND_PORT(c->pClipIn[1]);
                BIND_PORT(c->pClipOut[1]);
                BIND_PORT(c->pClipRed);
            }

            // Initialize horizontal axis values for each curve
            float delta = (meta::clipper::ODP_CURVE_DB_MAX - meta::clipper::ODP_CURVE_DB_MIN) / (meta::clipper::CURVE_MESH_POINTS-1);
            for (size_t i=0; i<meta::clipper::CURVE_MESH_POINTS; ++i)
                vOdp[i]         = dspu::db_to_gain(meta::clipper::ODP_CURVE_DB_MIN + delta * i);

            delta       = (meta::clipper::CLIP_CURVE_DB_MAX - meta::clipper::CLIP_CURVE_DB_MIN) / (meta::clipper::CURVE_MESH_POINTS-1);
            for (size_t i=0; i<meta::clipper::CURVE_MESH_POINTS; ++i)
                vLogSigmoid[i]  = dspu::db_to_gain(meta::clipper::CLIP_CURVE_DB_MIN + delta * i);

            delta       = (meta::clipper::CLIP_CURVE_X_MAX - meta::clipper::CLIP_CURVE_X_MIN) / (meta::clipper::CURVE_MESH_POINTS-1);
            for (size_t i=0; i<meta::clipper::CURVE_MESH_POINTS; ++i)
                vLinSigmoid[i]  = meta::clipper::CLIP_CURVE_X_MIN + delta * i;

            delta       = meta::clipper::TIME_HISTORY_MAX / (meta::clipper::TIME_MESH_POINTS - 1);
            for (size_t i=0; i<meta::clipper::TIME_MESH_POINTS; ++i)
                vTime[i]    = meta::clipper::TIME_HISTORY_MAX - i*delta;

            delta       = meta::clipper::WAVEFORM_HISTORY_MAX / (meta::clipper::TIME_MESH_POINTS - 1);
            for (size_t i=0; i<meta::clipper::TIME_MESH_POINTS; ++i)
                vWaveformTime[i]    = meta::clipper::WAVEFORM_HISTORY_MAX - i*delta;
        }

        void clipper::update_sample_rate(long sr)
        {
            const size_t max_odp_delay      = dspu::millis_to_samples(sr, meta::clipper::ODP_REACT_MAX) * 0.5f;
            const size_t samples_per_dot    = dspu::seconds_to_samples(
                sr, meta::clipper::TIME_HISTORY_MAX / meta::clipper::TIME_MESH_POINTS);
            const size_t wf_samples_per_dot    = dspu::seconds_to_samples(
                sr, meta::clipper::WAVEFORM_HISTORY_MAX / meta::clipper::TIME_MESH_POINTS);

            sInMeter.set_sample_rate(sr);
            sOutMeter.set_sample_rate(sr);
            sLufs.sMeter.set_sample_rate(sr);
            sLufs.sGain.set_sample_rate(sr);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->sBypass.init(sr);
                c->sDryDelay.init(max_odp_delay);
                c->sScDelay.init(max_odp_delay);
                c->sSc.init(1, meta::clipper::ODP_REACT_MAX);
                c->sSc.set_sample_rate(sr);
                c->sInGraph.init(meta::clipper::TIME_MESH_POINTS, samples_per_dot);
                c->sOutGraph.init(meta::clipper::TIME_MESH_POINTS, samples_per_dot);
                c->sRedGraph.init(meta::clipper::TIME_MESH_POINTS, samples_per_dot);
                c->sWaveformGraph.init(meta::clipper::TIME_MESH_POINTS, wf_samples_per_dot);
            }
        }

        size_t clipper::decode_dithering(size_t mode)
        {
            switch (mode)
            {
                case meta::clipper::DITHER_7BIT:        return 7;
                case meta::clipper::DITHER_8BIT:        return 8;
                case meta::clipper::DITHER_11BIT:       return 11;
                case meta::clipper::DITHER_12BIT:       return 12;
                case meta::clipper::DITHER_15BIT:       return 15;
                case meta::clipper::DITHER_16BIT:       return 16;
                case meta::clipper::DITHER_23BIT:       return 23;
                case meta::clipper::DITHER_24BIT:       return 24;
                case meta::clipper::DITHER_NONE:
                default:
                    return 0;
            }
            return 0;
        }

        bool clipper::update_odp_params(odp_params_t *params)
        {
            const float threshold   = dspu::db_to_gain(params->pThreshold->value());
            const float knee        = dspu::db_to_gain(params->pKnee->value());

            if ((threshold == params->fThreshold) &&
                (knee == params->fKnee))
                return false;

            params->fThreshold      = threshold;
            params->fKnee           = knee;

            return true;
        }

        bool clipper::update_clip_params(clip_params_t *params)
        {
            dspu::sigmoid::function_t func = vSigmoidFunctions[size_t(params->pFunction->value())];
            const float threshold   = lsp_min(params->pThreshold->value(), 0.99f);
            const float dc_off      = params->pDCOffset->value() * 0.01f;
            const float pumping     = dspu::db_to_gain(params->pPumping->value());

            if ((func == params->pFunc) &&
                (threshold == params->fThreshold) &&
                (dc_off == params->fDCOffset) &&
                (pumping == params->fPumping))
                return false;

            params->pFunc           = func;
            params->fThreshold      = threshold;
            params->fDCOffset       = dc_off;
            params->fPumping        = pumping;
            params->fKnee           = 1.0f - threshold;
            params->fScaling        = 1.0f / params->fKnee;

            return true;
        }

        void clipper::calc_odp_compressor(compressor_t *c, const odp_params_t *params)
        {
            const float th  = params->fThreshold;
            const float kn  = params->fKnee;

            c->x0           = th;
            c->x1           = th / kn;
            c->x2           = th * kn;

            float y1        = c->x1;
            float y2        = th;
            float dy        = y2 - y1;
            float dx1       = 1.0f/(c->x2 - c->x1);
            float dx2       = dx1*dx1;

            float k         = 1.0f;

            c->c            = k;
            c->b            = (3.0 * dy) * dx2 - (2.0 * k)*dx1;
            c->a            = (k - (2.0*dy)*dx1)*dx2;
        }

        float clipper::odp_curve(const compressor_t *c, float x)
        {
            if (x >= c->x2)
                return c->x0;
            if (x <= c->x1)
                return x;

            float v    = x - c->x1;
            return ((v * c->a + c->b) * v + c->c)*v + c->x1;
        }

        float clipper::odp_gain(const compressor_t *c, float x)
        {
            if (x >= c->x2)
                return c->x0 / x;
            if (x <= c->x1)
                return 1.0f;

            float v    = x - c->x1;
            return (((v * c->a + c->b) * v + c->c)*v + c->x1)/x;
        }

        void clipper::odp_curve(float *dst, const float *x, const compressor_t *c, size_t count)
        {
            for (size_t i=0; i<count; ++i)
                dst[i]      = odp_curve(c, x[i]);
        }

        void clipper::odp_gain(float *dst, const float *x, const compressor_t *c, size_t count)
        {
            for (size_t i=0; i<count; ++i)
                dst[i]      = odp_gain(c, x[i]);
        }

        void clipper::odp_link(float *dst, const float *src, float link, size_t count)
        {
            const float rlink = 1.0f - link;
            for (size_t i=0; i<count; ++i)
            {
                const float k = rlink + src[i] * link;
                dst[i]  = dst[i] * k;
            }
        }

        float clipper::clip_curve(const clip_params_t *p, float x)
        {
            float s = x * p->fPumping;
            if (s > p->fThreshold)
            {
                s               = (s - p->fThreshold) * p->fScaling;
                return p->pFunc(s) * p->fKnee + p->fThreshold;
            }
            else if (s < -p->fThreshold)
            {
                s               = (s + p->fThreshold) * p->fScaling;
                return p->pFunc(s) * p->fKnee - p->fThreshold;
            }

            return s;
        }

        void clipper::clip_curve(float *dst, const float *x, const clip_params_t *p, size_t count)
        {
            for (size_t i=0; i<count; ++i)
                dst[i]          = clip_curve(p, x[i]);
        }

        void clipper::update_settings()
        {
            const bool bypass       = pBypass->value() >= 0.5f;
            const size_t dither_bits= decode_dithering(pDithering->value());
            fThresh                 = dspu::db_to_gain(-pThresh->value());

            fInGain                 = pGainIn->value() * fThresh;
            fOutGain                = pGainOut->value();
            nFlags                  = lsp_setflag(nFlags, CF_BOOSTING, pBoosting->value() >= 0.5f);

            // Enable/disable input loudness limiter
            nFlags                  = lsp_setflag(nFlags, CF_LUFS_LIMITER, sLufs.pOn->value() >= 0.5f);
            sLufs.sGain.set_threshold(dspu::db_to_gain(sLufs.pThreshold->value()));

            fStereoLink             = (pStereoLink != NULL) ? pStereoLink->value() * 0.01f : 1.0f;
            nFlags                  = lsp_setflag(nFlags, CF_ODP_ENABLED, sOdp.pOn->value() >= 0.5f);
            if (update_odp_params(&sOdp))
            {
                calc_odp_compressor(&sComp, &sOdp);
                nFlags                 |= CF_SYNC_ODP;
            }
            nFlags                  = lsp_setflag(nFlags, CF_CLIP_ENABLED, sClip.pOn->value() >= 0.5f);
            nFlags                  = lsp_setflag(nFlags, CF_DC_COMPENSATE, sClip.pDCCompensate->value() >= 0.5f);
            if (update_clip_params(&sClip))
                nFlags                 |= CF_SYNC_CLIP;

            // Adjust the compensation delays
            const float reactivity      = sOdp.pReactivity->value();
            const size_t latency        = dspu::millis_to_samples(fSampleRate, reactivity) * 0.5f;

            // Compute the final latency
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                // Manage flags
                c->nFlags               = lsp_setflag(c->nFlags, CH_IN_GRAPH, c->pInVisible->value() >= 0.5f);
                c->nFlags               = lsp_setflag(c->nFlags, CH_OUT_GRAPH, c->pOutVisible->value() >= 0.5f);
                c->nFlags               = lsp_setflag(c->nFlags, CH_RED_GRAPH, c->pRedVisible->value() >= 0.5f);

                // Update sidechain reactivity
                c->sSc.set_reactivity(reactivity);
                c->sSc.set_mode(dspu::SCM_RMS);
                c->sSc.set_stereo_mode(dspu::SCSM_STEREO);

                c->sBypass.set_bypass(bypass);
                c->sDither.set_bits(dither_bits);
                c->sScDelay.set_delay(latency);
                c->sDryDelay.set_delay(latency);
            }
            set_latency(latency);
        }

        void clipper::bind_input_buffers()
        {
            sLufs.fIn           = GAIN_AMP_M_INF_DB;
            sLufs.fRed          = GAIN_AMP_P_72_DB;

            fInLufs             = GAIN_AMP_M_INF_DB;
            fOutLufs            = GAIN_AMP_M_INF_DB;

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                c->vIn              = c->pDataIn->buffer<float>();
                c->vOut             = c->pDataOut->buffer<float>();

                c->fIn              = GAIN_AMP_M_INF_DB;
                c->fOut             = GAIN_AMP_M_INF_DB;
                c->fRed             = GAIN_AMP_P_72_DB;

                c->fOdpIn           = GAIN_AMP_M_INF_DB;
                c->fOdpOut          = GAIN_AMP_M_INF_DB;
                c->fOdpRed          = GAIN_AMP_P_72_DB;

                c->fClipIn[0]       = GAIN_AMP_M_INF_DB;
                c->fClipIn[1]       = GAIN_AMP_M_INF_DB;
                c->fClipOut[0]      = GAIN_AMP_M_INF_DB;
                c->fClipOut[1]      = GAIN_AMP_M_INF_DB;
                c->fClipRed         = GAIN_AMP_P_72_DB;
            }
        }

        void clipper::advance_buffers(size_t samples)
        {
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                c->vIn             += samples;
                c->vOut            += samples;
            }
        }

        void clipper::process_odp_channel(channel_t *c, size_t samples)
        {
            if (!(nFlags & CF_ODP_ENABLED))
            {
                dsp::fill_one(c->vSc, samples);

                c->fOdpIn               = GAIN_AMP_M_INF_DB;
                c->fOdpOut              = GAIN_AMP_M_INF_DB;
                c->fOdpRed              = GAIN_AMP_0_DB;

                return;
            }

            // Measure input signal
            const size_t odp_idx    = dsp::abs_max_index(c->vSc, samples);
            const float odp_in      = c->vSc[odp_idx];

            // Apply ODP
            odp_gain(c->vSc, c->vSc, &sComp, samples);
            dsp::mul2(c->vData, c->vSc, samples);

            // Measure output
            const float odp_red     = c->vSc[odp_idx];
            const float odp_out     = odp_in * odp_red;

            c->fOdpIn               = lsp_max(c->fOdpIn, odp_in);
            c->fOdpOut              = lsp_max(c->fOdpOut, odp_out);
            c->fOdpRed              = lsp_min(c->fOdpRed, odp_red);
        }

        void clipper::process_clip_channel(channel_t *c, size_t samples)
        {
            // Check that clipping is enabled
            if (!(nFlags & CF_CLIP_ENABLED))
            {
                c->fClipIn[0]           = GAIN_AMP_M_INF_DB;
                c->fClipIn[1]           = GAIN_AMP_M_INF_DB;
                c->fClipOut[0]          = GAIN_AMP_M_INF_DB;
                c->fClipOut[1]          = GAIN_AMP_M_INF_DB;
                c->fClipRed             = GAIN_AMP_0_DB;

                return;
            }

            // Do clipping
            const float dc      = sClip.fDCOffset;
            size_t clip_idx[2];
            float clip_in[2], clip_out[2];

            if (dc != 0.0f)
            {
                // Apply DC offset
                dsp::add_k2(c->vData, dc, samples);

                // Perform clipping
                dsp::minmax_index(c->vData, samples, &clip_idx[0], &clip_idx[1]);
                clip_in[0]          = fabsf(c->vData[clip_idx[0]]);
                clip_in[1]          = fabsf(c->vData[clip_idx[1]]);

                clip_curve(c->vData, c->vData, &sClip, samples);

                clip_out[0]         = fabsf(c->vData[clip_idx[0]]);
                clip_out[1]         = fabsf(c->vData[clip_idx[1]]);

                // Compensate DC offset if needed
                if (nFlags & CF_DC_COMPENSATE)
                    dsp::sub_k2(c->vData, dc, samples);
            }
            else
            {
                clip_idx[0]         = dsp::abs_max_index(c->vData, samples);
                clip_idx[1]         = clip_idx[0];
                clip_in[0]          = fabsf(c->vData[clip_idx[0]]);
                clip_in[1]          = clip_in[0];

                clip_curve(c->vData, c->vData, &sClip, samples);

                clip_out[0]         = fabsf(c->vData[clip_idx[0]]);
                clip_out[1]         = clip_out[0];
            }

            // Measure input and output level
            if (clip_in[0] > c->fClipIn[0])
            {
                c->fClipIn[0]           = clip_in[0];
                c->fClipOut[0]          = clip_out[0];
            }
            if (clip_in[1] > c->fClipIn[1])
            {
                c->fClipIn[1]           = clip_in[1];
                c->fClipOut[1]          = clip_out[1];
            }

            // Compute gain reduction
            const size_t imax       = (clip_in[0] >= clip_in[1]) ? 0 : 1;
            const float clip_red    = (clip_in[imax] >= GAIN_AMP_M_120_DB) ? clip_out[imax] / clip_in[imax] : GAIN_AMP_0_DB;
            c->fClipRed             = lsp_min(c->fClipRed, clip_red);
        }

        void clipper::meter_channel(channel_t *c, size_t samples)
        {
            // Compute reduction buffer
            for (size_t i=0; i<samples; ++i)
                c->vRedMeter[i]         = (c->vInMeter[i] >= GAIN_AMP_M_120_DB) ? fabsf(c->vData[i]) / c->vInMeter[i] : GAIN_AMP_0_DB;

            // Update graphs
            c->sInGraph.process(c->vInMeter, samples);
            c->sOutGraph.process(c->vData, samples);
            c->sRedGraph.process(c->vRedMeter, samples);
            c->sWaveformGraph.process(c->vData, samples);

            // Update momentary values
            const float in          = dsp::max(c->vInMeter, samples);
            const float out         = dsp::abs_max(c->vData, samples);
            const float red         = dsp::min(c->vRedMeter, samples);

            c->fIn                  = lsp_max(c->fIn, in);
            c->fOut                 = lsp_max(c->fOut, out);
            c->fRed                 = lsp_min(c->fRed, red);
        }

        void clipper::process_clipper(size_t samples)
        {
            if (nChannels > 1)
            {
                // Stereo version
                channel_t *l            = &vChannels[0];
                channel_t *r            = &vChannels[1];

                // Measure input loudness
                dsp::mul_k3(l->vData, l->vIn, fInGain, samples);
                dsp::mul_k3(r->vData, r->vIn, fInGain, samples);

                sLufs.sMeter.bind(0, NULL, l->vData);
                sLufs.sMeter.bind(1, NULL, r->vData);
                sLufs.sMeter.process(vBuffer, samples);

                size_t max_index        = dsp::abs_max_index(vBuffer, samples);
                sLufs.fIn               = lsp_max(sLufs.fIn, vBuffer[max_index]);

                // Apply LUFS limiter
                if (nFlags & CF_LUFS_LIMITER)
                {
                    sLufs.sGain.process(vBuffer, vBuffer, samples);
                    sLufs.fRed              = lsp_min(sLufs.fRed, vBuffer[max_index]);

                    dsp::mul2(l->vData, vBuffer, samples);
                    dsp::mul2(r->vData, vBuffer, samples);
                }
                else
                    sLufs.fRed              = GAIN_AMP_0_DB;

                // Process sidechain signal
                if (fStereoLink >= 1.0f)
                {
                    dsp::lr_to_mid(r->vSc, l->vData, r->vData, samples);
                    l->sSc.process(l->vSc, const_cast<const float **>(&r->vSc), samples);
                    r->sSc.process(r->vSc, const_cast<const float **>(&r->vSc), samples);
                }
                else if (fStereoLink > 0.0f)
                {
                    dsp::mix_copy2(l->vSc, l->vData, r->vData, 1.0f - fStereoLink * 0.5f, fStereoLink * 0.5f, samples);
                    dsp::mix_copy2(r->vSc, l->vData, r->vData, fStereoLink * 0.5f, 1.0f - fStereoLink * 0.5f, samples);

                    l->sSc.process(l->vSc, const_cast<const float **>(&l->vSc), samples);
                    r->sSc.process(r->vSc, const_cast<const float **>(&r->vSc), samples);
                }
                else
                {
                    l->sSc.process(l->vSc, const_cast<const float **>(&l->vData), samples);
                    r->sSc.process(r->vSc, const_cast<const float **>(&r->vData), samples);
                }
                l->sScDelay.process(l->vData, l->vData, samples);
                r->sScDelay.process(r->vData, r->vData, samples);

                // Remember signal at the input of the band for metering
                dsp::abs2(l->vInMeter, l->vData, samples);
                dsp::abs2(r->vInMeter, r->vData, samples);

                // Apply overdrive protection
                process_odp_channel(l, samples);
                process_odp_channel(r, samples);

                // Apply clipping
                process_clip_channel(l, samples);
                process_clip_channel(r, samples);

                // Perform output metering
                meter_channel(l, samples);
                meter_channel(r, samples);

                // Apply gain boosting compensation
                if (!(nFlags & CF_BOOSTING))
                {
                    dsp::mul_k2(l->vData, 1.0f / fThresh, samples);
                    dsp::mul_k2(r->vData, 1.0f / fThresh, samples);
                }
            }
            else
            {
                // Mono version
                channel_t *c            = &vChannels[0];

                // Measure input loudness
                dsp::mul_k3(c->vData, c->vIn, fInGain, samples);

                sLufs.sMeter.bind(0, NULL, c->vData);
                sLufs.sMeter.process(vBuffer, samples);

                size_t max_index        = dsp::abs_max_index(vBuffer, samples);
                sLufs.fIn               = lsp_max(sLufs.fIn, vBuffer[max_index]);

                // Apply LUFS limiter
                if (nFlags & CF_LUFS_LIMITER)
                {
                    sLufs.sGain.process(vBuffer, vBuffer, samples);
                    sLufs.fRed              = lsp_min(sLufs.fRed, vBuffer[max_index]);

                    dsp::mul2(c->vData, vBuffer, samples);
                }
                else
                    sLufs.fRed              = GAIN_AMP_0_DB;

                // Process sidechain signal
                c->sSc.process(c->vSc, const_cast<const float **>(&c->vData), samples);
                c->sScDelay.process(c->vData, c->vData, samples);

                // Remember signal at the input of the band for metering
                dsp::abs2(c->vInMeter, c->vData, samples);

                // Apply overdrive protection
                process_odp_channel(c, samples);

                // Apply clipping
                process_clip_channel(c, samples);

                // Perform output metering
                meter_channel(c, samples);

                // Apply gain boosting compensation
                if (!(nFlags & CF_BOOSTING))
                    dsp::mul_k2(c->vData, 1.0f / fThresh, samples);
            }
        }

        void clipper::output_signal(size_t samples)
        {
            // Process the signal
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                dsp::mul_k2(c->vData, fOutGain, samples);
                c->sDither.process(c->vData, c->vData, samples);
                sOutMeter.bind(i, NULL, c->vData);
                sInMeter.bind(i, NULL, c->vIn);

                c->sDryDelay.process(vBuffer, c->vIn, samples);
                c->sBypass.process(c->vOut, vBuffer, c->vData, samples);
            }

            // Measure input and output loudness
            sInMeter.process(vBuffer, samples);
            fInLufs             = lsp_max(fOutLufs, dsp::abs_max(vBuffer, samples));

            sOutMeter.process(vBuffer, samples);
            fOutLufs            = lsp_max(fOutLufs, dsp::abs_max(vBuffer, samples));
        }

        void clipper::output_meters()
        {
            sLufs.pIn->set_value(dspu::gain_to_lufs(sLufs.fIn));
            sLufs.pRed->set_value(sLufs.fRed);

            pLufsIn->set_value(dspu::gain_to_lufs(fInLufs));
            pLufsOut->set_value(dspu::gain_to_lufs(fOutLufs));

            uint32_t flags = 0;

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];
                flags          |= (c->nFlags & (CH_IN_GRAPH | CH_OUT_GRAPH | CH_RED_GRAPH));

                const float out = (nFlags & CF_BOOSTING) ? c->fOut : c->fOut / fThresh;

                c->pIn->set_value(c->fIn / fThresh);
                c->pOut->set_value(out);
                c->pRed->set_value(c->fRed);

                c->pOdpIn->set_value(c->fOdpIn);
                c->pOdpOut->set_value(c->fOdpOut);
                c->pOdpRed->set_value(c->fOdpRed);

                c->pClipIn[0]->set_value(c->fClipIn[0]);
                c->pClipIn[1]->set_value(c->fClipIn[1]);
                c->pClipOut[0]->set_value(c->fClipOut[0]);
                c->pClipOut[1]->set_value(c->fClipOut[1]);
                c->pClipRed->set_value(c->fClipRed);
            }

            if (flags != 0)
                pWrapper->query_display_draw();
        }

        void clipper::output_mesh_curves(size_t samples)
        {
            plug::mesh_t *mesh  = NULL;

            // Sync ODP curve
            if (nFlags & CF_SYNC_ODP)
            {
                mesh                = (sOdp.pCurveMesh != NULL) ? sOdp.pCurveMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    dsp::copy(mesh->pvData[0], vOdp, meta::clipper::CURVE_MESH_POINTS);
                    odp_curve(mesh->pvData[1], vOdp, &sComp, meta::clipper::CURVE_MESH_POINTS);
                    mesh->data(2, meta::clipper::CURVE_MESH_POINTS);

                    // Mark mesh as synchronized
                    nFlags             &= uint32_t(~CF_SYNC_ODP);
                }
            }

            // Sync sigmoid curve
            if (nFlags & CF_SYNC_CLIP)
            {
                mesh                = (sClip.pCurveMesh != NULL) ? sClip.pCurveMesh->buffer<plug::mesh_t>() : NULL;
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    dsp::copy(mesh->pvData[0], vLinSigmoid, meta::clipper::CURVE_MESH_POINTS);
                    clip_curve(mesh->pvData[1], vLinSigmoid, &sClip, meta::clipper::CURVE_MESH_POINTS);
                    dsp::copy(mesh->pvData[2], vLogSigmoid, meta::clipper::CURVE_MESH_POINTS);
                    clip_curve(mesh->pvData[3], vLogSigmoid, &sClip, meta::clipper::CURVE_MESH_POINTS);
                    mesh->data(4, meta::clipper::CURVE_MESH_POINTS);

                    // Mark mesh as synchronized
                    nFlags             &= uint32_t(~CF_SYNC_CLIP);
                }
            }

            // Output oscilloscope graphs for output clipper
            mesh            = pTimeMesh->buffer<plug::mesh_t>();
            if ((mesh != NULL) && (mesh->isEmpty()))
            {
                size_t index    = 0;
                float *t        = mesh->pvData[index++];

                dsp::copy(&t[2], vTime, meta::clipper::TIME_MESH_POINTS);
                t[0]            = t[2] + meta::clipper::TIME_HISTORY_GAP;
                t[1]            = t[0];
                t              += meta::clipper::TIME_MESH_POINTS + 2;
                t[0]            = t[-1] - meta::clipper::TIME_HISTORY_GAP;
                t[1]            = t[0];

                // Output data for each channel
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];

                    // Fill time values
                    float *in       = mesh->pvData[index++];
                    float *out      = mesh->pvData[index++];
                    float *red      = mesh->pvData[index++];

                    dsp::copy(&in[2], c->sInGraph.data(), meta::clipper::TIME_MESH_POINTS);
                    dsp::copy(&out[2], c->sOutGraph.data(), meta::clipper::TIME_MESH_POINTS);
                    dsp::copy(&red[2], c->sRedGraph.data(), meta::clipper::TIME_MESH_POINTS);

                    // Generate extra points
                    in[0]           = 0.0f;
                    in[1]           = in[2];
                    out[0]          = out[2];
                    out[1]          = out[2];
                    red[0]          = red[2];
                    red[1]          = red[2];

                    in             += meta::clipper::TIME_MESH_POINTS + 2;
                    out            += meta::clipper::TIME_MESH_POINTS + 2;
                    red            += meta::clipper::TIME_MESH_POINTS + 2;

                    in[0]           = in[-1];
                    in[1]           = 0.0f;
                    out[0]          = out[-1];
                    out[1]          = out[-1];
                    red[0]          = red[-1];
                    red[1]          = red[-1];
                }

                // Notify mesh contains data
                mesh->data(index, meta::clipper::TIME_MESH_POINTS + 4);
            }

            // Output oscilloscope graphs for output clipper
            mesh            = pWaveformMesh->buffer<plug::mesh_t>();
            if ((mesh != NULL) && (mesh->isEmpty()))
            {
                size_t index    = 0;
                float *t        = mesh->pvData[index++];

                dsp::copy(&t[2], vWaveformTime, meta::clipper::TIME_MESH_POINTS);
                t[0]            = t[2] + meta::clipper::TIME_HISTORY_GAP;
                t[1]            = t[0];
                t              += meta::clipper::TIME_MESH_POINTS + 2;
                t[0]            = t[-1] - meta::clipper::TIME_HISTORY_GAP;
                t[1]            = t[0];

                // Output data for each channel
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = &vChannels[i];

                    // Fill time values
                    float *osc      = mesh->pvData[index++];

                    dsp::copy(&osc[2], c->sWaveformGraph.data(), meta::clipper::TIME_MESH_POINTS);

                    // Generate extra points
                    osc[0]          = 0.0f;
                    osc[1]          = osc[2];
                    osc            += meta::clipper::TIME_MESH_POINTS + 2;
                    osc[0]          = osc[-1];
                    osc[1]          = 0.0f;
                }

                // Notify mesh contains data
                mesh->data(index, meta::clipper::TIME_MESH_POINTS + 4);
            }
        }

        void clipper::process(size_t samples)
        {
            bind_input_buffers();

            for (size_t offset = 0; offset < samples; )
            {
                size_t to_do    = lsp_min(samples - offset, BUFFER_SIZE);

                process_clipper(to_do);
                output_signal(to_do);

                advance_buffers(to_do);
                offset         += to_do;
            }

            output_meters();
            output_mesh_curves(samples);
        }

        void clipper::ui_activated()
        {
            nFlags             |= CF_SYNC_ALL;
        }

        bool clipper::inline_display(plug::ICanvas *cv, size_t width, size_t height)
        {
            // Check proportions
            if (height > (M_RGOLD_RATIO * width))
                height  = M_RGOLD_RATIO * width;

            // Init canvas
            if (!cv->init(width, height))
                return false;
            width   = cv->width();
            height  = cv->height();

            // Clear background
            bool bypassing = vChannels[0].sBypass.bypassing();
            cv->set_color_rgb((bypassing) ? CV_DISABLED : CV_BACKGROUND);
            cv->paint();

            // Calc axis params
            float zy    = 1.0f/GAIN_AMP_M_24_DB;
            float dx    = -float(width/meta::clipper::TIME_HISTORY_MAX);
            float dy    = height/(logf(GAIN_AMP_M_24_DB)-logf(GAIN_AMP_P_12_DB));

            // Draw axis
            cv->set_line_width(1.0);

            // Draw vertical lines
            cv->set_color_rgb(CV_YELLOW, 0.5f);
            for (float i=1.0; i < (meta::clipper::TIME_HISTORY_MAX - 0.1f); i += 1.0f)
            {
                float ax = width + dx*i;
                cv->line(ax, 0, ax, height);
            }

            // Draw horizontal lines
            cv->set_color_rgb(CV_WHITE, 0.5f);
            for (float i=GAIN_AMP_M_18_DB; i<GAIN_AMP_P_12_DB; i *= GAIN_AMP_P_6_DB)
            {
                float ay = height + dy*(logf(i*zy));
                cv->line(0, ay, width, ay);
            }

            // Allocate buffer: t, x, y, in[0], out[0], gain[0], in[1], out[1], gain[1]
            const size_t count  = width + 4;
            pIDisplay           = core::IDBuffer::reuse(pIDisplay, 3 + nChannels * 3, count);
            core::IDBuffer *b   = pIDisplay;
            if (b == NULL)
                return false;

            static const uint32_t c_colors[] = {
                CV_MIDDLE_CHANNEL_IN, CV_MIDDLE_CHANNEL, CV_BRIGHT_BLUE,
                CV_LEFT_CHANNEL_IN, CV_RIGHT_CHANNEL_IN,
                CV_LEFT_CHANNEL, CV_RIGHT_CHANNEL,
                CV_BRIGHT_BLUE, CV_BRIGHT_BLUE
            };

            const uint32_t *col = (nChannels > 1) ? &c_colors[3] : c_colors;
            float r             = meta::clipper::TIME_MESH_POINTS/float(width);

            // Fill time array
            float *t            = &b->v[0][2];
            for (size_t j=0; j<width; ++j)
            {
                size_t k        = r*j;
                t[j]            = vTime[k];
            }
            t[-2]           = t[0] + meta::clipper::TIME_HISTORY_GAP;
            t[-1]           = t[-2];
            t              += width;
            t[0]            = t[-1] - meta::clipper::TIME_HISTORY_GAP;
            t[1]            = t[0];

            cv->set_line_width(2.0f);

            // Step 1: Fill meshes
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];

                // Initialize values
                const float *in_g   = c->sInGraph.data();
                const float *out_g  = c->sOutGraph.data();
                const float *red_g  = c->sRedGraph.data();
                float *in           = &b->v[3 + i*3 + 0][2];
                float *out          = &b->v[3 + i*3 + 1][2];
                float *red          = &b->v[3 + i*3 + 2][2];

                for (size_t k=0; k<width; ++k)
                {
                    size_t off      = size_t(r*k);
                    in[k]           = in_g[off];
                    out[k]          = out_g[off];
                    red[k]          = red_g[off];
                }

                // Add terminating points
                in[-2]          = 0.0f;
                in[-1]          = in[0];
                out[-2]         = out[0];
                out[-1]         = out[0];
                red[-2]         = red[0];
                red[-1]         = red[0];

                in             += width;
                out            += width;
                red            += width;

                in[0]           = in[-1];
                in[1]           = 0.0f;
                out[0]          = out[-1];
                out[1]          = out[-1];
                red[0]          = red[-1];
                red[1]          = red[-1];
            }

            // Step 2: Draw input meshes
            for (size_t i=0; i<nChannels; ++i, ++col)
            {
                channel_t *c    = &vChannels[i];
                if (!(c->nFlags & CH_IN_GRAPH))
                    continue;

                // Initialize coords
                dsp::fill(b->v[1], width, count);  // x = width
                dsp::fill(b->v[2], height, count); // y = height
                dsp::fmadd_k3(b->v[1], b->v[0], dx, count); // x[i] = width - dx * t[i]
                dsp::axis_apply_log1(b->v[2], b->v[3 + i*3 + 0], zy, dy, count);

                // Draw channel
                uint32_t color = (bypassing) ? CV_SILVER : *col;
                Color stroke(color), fill(color, 0.5f);
                cv->draw_poly(b->v[1], b->v[2], count, stroke, fill);
            }

            // Step 2: Draw output meshes
            for (size_t i=0; i<nChannels; ++i, ++col)
            {
                channel_t *c    = &vChannels[i];
                if (!(c->nFlags & CH_OUT_GRAPH))
                    continue;

                // Initialize coords
                dsp::fill(b->v[1], width, count);  // x = width
                dsp::fill(b->v[2], height, count); // y = height
                dsp::fmadd_k3(b->v[1], b->v[0], dx, count); // x[i] = width - dx * t[i]
                dsp::axis_apply_log1(b->v[2], b->v[3 + i*3 + 1], zy, dy, count);

                // Draw channel
                uint32_t color = (bypassing) ? CV_SILVER : *col;
                cv->set_color_rgb(color);
                cv->draw_lines(b->v[1], b->v[2], width);
            }

            // Step 2: Draw gain
            for (size_t i=0; i<nChannels; ++i, ++col)
            {
                channel_t *c    = &vChannels[i];
                if (!(c->nFlags & CH_RED_GRAPH))
                    continue;

                // Initialize coords
                dsp::fill(b->v[1], width, count);  // x = width
                dsp::fill(b->v[2], height, count); // y = height
                dsp::fmadd_k3(b->v[1], b->v[0], dx, count); // x[i] = width - dx * t[i]
                dsp::axis_apply_log1(b->v[2], b->v[3 + i*3 + 2], zy, dy, count);

                // Draw channel
                uint32_t color = (bypassing) ? CV_SILVER : *col;
                cv->set_color_rgb(color);
                cv->draw_lines(b->v[1], b->v[2], width);
            }

            return true;
        }

        void clipper::dump(dspu::IStateDumper *v) const
        {
            plug::Module::dump(v);

            v->write("nChannels", nChannels);

            v->begin_array("vChannels", vChannels, 2);
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    const channel_t *c = &vChannels[i];

                    v->begin_object(c, sizeof(channel_t));
                    {
                        v->write_object("sBypass", &c->sBypass);
                        v->write_object("sDryDelay", &c->sDryDelay);
                        v->write_object("sScDelay", &c->sScDelay);
                        v->write_object("sSc", &c->sSc);
                        v->write_object("sDither", &c->sDither);
                        v->write_object("sInGraph", &c->sInGraph);
                        v->write_object("sOutGraph", &c->sOutGraph);
                        v->write_object("sRedGraph", &c->sOutGraph);

                        v->write("nFlags", c->nFlags);

                        v->write("fIn", c->fIn);
                        v->write("fOut", c->fOut);
                        v->write("fRed", c->fRed);

                        v->write("fOdpIn", c->fOdpIn);
                        v->write("fOdpOut", c->fOdpOut);
                        v->write("fOdpRed", c->fOdpRed);

                        v->writev("fClipIn", c->fClipIn, 2);
                        v->writev("fClipOut", c->fClipOut, 2);
                        v->write("fClipRed", c->fClipRed);

                        v->write("vIn", c->vIn);
                        v->write("vOut", c->vOut);
                        v->write("vInMeter", c->vInMeter);
                        v->write("vRedMeter", c->vRedMeter);
                        v->write("vData", c->vData);
                        v->write("vSc", c->vSc);

                        v->write("pDataIn", c->pDataIn);
                        v->write("pDataOut", c->pDataOut);

                        v->write("pInVisible", c->pInVisible);
                        v->write("pOutVisible", c->pOutVisible);
                        v->write("pRedVisible", c->pRedVisible);

                        v->write("pIn", c->pIn);
                        v->write("pOut", c->pOut);
                        v->write("pRed", c->pRed);

                        v->write("pOdpIn", c->pOdpIn);
                        v->write("pOdpOut", c->pOdpOut);
                        v->write("pOdpRed", c->pOdpRed);

                        v->writev("pClipIn", c->pClipIn, 2);
                        v->writev("pClipOut", c->pClipOut, 2);
                        v->write("pClipRed", c->pClipRed);
                    }
                    v->end_object();
                }
            }
            v->end_array();

            v->write_object("sInMeter", &sInMeter);
            v->write_object("sOutMeter", &sOutMeter);

            v->begin_object("sComp", &sComp, sizeof(compressor_t));
            {
                const compressor_t *c = &sComp;

                v->write("x0", c->x0);
                v->write("x1", c->x1);
                v->write("x2", c->x2);
                v->write("t", c->t);
                v->write("a", c->a);
                v->write("b", c->b);
                v->write("c", c->c);
            }
            v->end_object();

            v->begin_object("sOdp", &sOdp, sizeof(odp_params_t));
            {
                const odp_params_t *o = &sOdp;

                v->write("fThreshold", o->fThreshold);
                v->write("fKnee", o->fKnee);

                v->write("pOn", o->pOn);
                v->write("pThreshold", o->pThreshold);
                v->write("pKnee", o->pKnee);
                v->write("pReactivity", o->pReactivity);
                v->write("pCurveMesh", o->pCurveMesh);
            }
            v->end_object();

            v->begin_object("sClip", &sClip, sizeof(clip_params_t));
            {
                const clip_params_t *c = &sClip;

                v->write("pFunc", c->pFunc);
                v->write("fThreshold", c->fThreshold);
                v->write("fDCOffset", c->fDCOffset);
                v->write("fPumping", c->fPumping);
                v->write("fScaling", c->fScaling);
                v->write("fKnee", c->fKnee);

                v->write("pOn", c->pOn);
                v->write("pFunction", c->pFunction);
                v->write("pThreshold", c->pThreshold);
                v->write("pDCOffset", c->pDCOffset);
                v->write("pDCCompensate", c->pDCCompensate);
                v->write("pPumping", c->pPumping);
                v->write("pCurveMesh", c->pCurveMesh);
            }
            v->end_object();

            v->begin_object("sLufs", &sLufs, sizeof(lufs_limiter_t));
            {
                const lufs_limiter_t *l = &sLufs;

                v->write_object("sMeter", &l->sMeter);
                v->write_object("sGain", &l->sGain);

                v->write("fIn", l->fIn);
                v->write("fRed", l->fRed);

                v->write("pOn", l->pOn);
                v->write("pIn", l->pIn);
                v->write("pRed", l->pRed);
                v->write("pThreshold", l->pThreshold);
            }
            v->end_object();

            v->write("fInGain", fInGain);
            v->write("fOutGain", fOutGain);
            v->write("fInLufs", fInLufs);
            v->write("fOutLufs", fOutLufs);
            v->write("fThresh", fThresh);
            v->write("fStereoLink", fStereoLink);
            v->write("nFlags", nFlags);

            v->write("vBuffer", vBuffer);
            v->write("vOdp", vOdp);
            v->write("vLinSigmoid", vLinSigmoid);
            v->write("vLogSigmoid", vLogSigmoid);
            v->write("vTime", vTime);
            v->write("vWaveformTime", vWaveformTime);
            v->write("pIDisplay", pIDisplay);

            v->write("pBypass", pBypass);
            v->write("pGainIn", pGainIn);
            v->write("pGainOut", pGainOut);
            v->write("pLufsIn", pLufsIn);
            v->write("pLufsOut", pLufsOut);
            v->write("pThresh", pThresh);
            v->write("pBoosting", pBoosting);
            v->write("pStereoLink", pStereoLink);
            v->write("pDithering", pDithering);
            v->write("pTimeMesh", pTimeMesh);

            v->write("pData", pData);
        }

    } /* namespace plugins */
} /* namespace lsp */


