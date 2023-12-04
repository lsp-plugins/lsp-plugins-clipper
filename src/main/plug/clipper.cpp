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

#include <lsp-plug.in/common/alloc.h>
#include <lsp-plug.in/common/debug.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/plug-fw/meta/func.h>
#include <lsp-plug.in/shared/debug.h>

#include <private/plugins/clipper.h>

/* The size of temporary buffer for audio processing */
#define BUFFER_SIZE         0x400U

namespace lsp
{
    namespace plugins
    {
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
            sClip.fPumping          = 1.0f;
            sClip.fScaling          = 0.0f;
            sClip.fKnee             = 0.0f;

            sClip.pOn               = NULL;
            sClip.pFunction         = NULL;
            sClip.pThreshold        = NULL;
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
                nChannels * (
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

                c->sDither.init();

                c->fIn                  = GAIN_AMP_M_INF_DB;
                c->fOut                 = GAIN_AMP_M_INF_DB;
                c->fRed                 = GAIN_AMP_M_INF_DB;

                c->fOdpIn               = GAIN_AMP_M_INF_DB;
                c->fOdpOut              = GAIN_AMP_M_INF_DB;
                c->fOdpRed              = GAIN_AMP_M_INF_DB;

                c->fClipIn              = GAIN_AMP_M_INF_DB;
                c->fClipOut             = GAIN_AMP_M_INF_DB;
                c->fClipRed             = GAIN_AMP_M_INF_DB;

                c->vIn                  = NULL;
                c->vOut                 = NULL;
                c->vData                = advance_ptr_bytes<float>(ptr, szof_buffer);
                c->vSc                  = advance_ptr_bytes<float>(ptr, szof_buffer);

                // Initialize ports
                c->pDataIn              = NULL;
                c->pDataOut             = NULL;

                c->pIn                  = NULL;
                c->pOut                 = NULL;
                c->pRed                 = NULL;

                c->pOdpIn               = NULL;
                c->pOdpOut              = NULL;
                c->pOdpRed              = NULL;

                c->pClipIn              = NULL;
                c->pClipOut             = NULL;
                c->pClipRed             = NULL;

                c->pTimeMesh            = NULL;
            }

            lsp_assert( ptr <= tail );

            // Bind ports
            lsp_trace("Binding input ports");
            size_t port_id      = 0;

            // Bind input audio ports
            for (size_t i=0; i<nChannels; ++i)
                vChannels[i].pDataIn    = trace_port(ports[port_id++]);

            // Bind output audio ports
            for (size_t i=0; i<nChannels; ++i)
                vChannels[i].pDataOut   = trace_port(ports[port_id++]);

            // Bind common ports
            lsp_trace("Binding common ports");
            pBypass             = trace_port(ports[port_id++]);
            pGainIn             = trace_port(ports[port_id++]);
            pGainOut            = trace_port(ports[port_id++]);
            sLufs.pOn           = trace_port(ports[port_id++]);
            sLufs.pThreshold    = trace_port(ports[port_id++]);
            sLufs.pIn           = trace_port(ports[port_id++]);
            sLufs.pRed          = trace_port(ports[port_id++]);
            pLufsIn             = trace_port(ports[port_id++]);
            pLufsOut            = trace_port(ports[port_id++]);
            pThresh             = trace_port(ports[port_id++]);
            pBoosting           = trace_port(ports[port_id++]);
            pDithering          = trace_port(ports[port_id++]);
            trace_port(ports[port_id++]); // Skip clipper linear/logarithmic graph view

            // Bind clipper ports
            lsp_trace("Binding clipper ports");
            sOdp.pOn                = trace_port(ports[port_id++]);
            sOdp.pThreshold         = trace_port(ports[port_id++]);
            sOdp.pKnee              = trace_port(ports[port_id++]);
            sOdp.pReactivity        = trace_port(ports[port_id++]);
            sOdp.pCurveMesh         = trace_port(ports[port_id++]);
            sClip.pOn               = trace_port(ports[port_id++]);
            sClip.pFunction         = trace_port(ports[port_id++]);
            sClip.pThreshold        = trace_port(ports[port_id++]);
            sClip.pPumping          = trace_port(ports[port_id++]);
            sClip.pCurveMesh        = trace_port(ports[port_id++]);
            pStereoLink             = (nChannels > 1) ? trace_port(ports[port_id++]) : NULL;

            lsp_trace("Skipping graph visibility ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                trace_port(ports[port_id++]); // Skip input level graph visibility
                trace_port(ports[port_id++]); // Skip output level graph visibility
                trace_port(ports[port_id++]); // Skip gain reduction graph visibility
            }

            // Bind channel metering
            lsp_trace("Binding channel metering ports");
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->pIn                  = trace_port(ports[port_id++]);
                c->pOut                 = trace_port(ports[port_id++]);
                c->pRed                 = trace_port(ports[port_id++]);

                c->pOdpIn               = trace_port(ports[port_id++]);
                c->pOdpOut              = trace_port(ports[port_id++]);
                c->pOdpRed              = trace_port(ports[port_id++]);

                c->pClipIn              = trace_port(ports[port_id++]);
                c->pClipOut             = trace_port(ports[port_id++]);
                c->pClipRed             = trace_port(ports[port_id++]);

                c->pTimeMesh            = trace_port(ports[port_id++]);
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
        }

        void clipper::update_sample_rate(long sr)
        {
            const size_t max_odp_delay      = dspu::millis_to_samples(sr, meta::clipper::ODP_REACT_MAX) * 0.5f;
            const size_t samples_per_dot    = dspu::seconds_to_samples(
                sr, meta::clipper::TIME_HISTORY_MAX / meta::clipper::TIME_MESH_POINTS);

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
            const float pumping     = dspu::db_to_gain(params->pPumping->value());

            if ((func == params->pFunc) &&
                (threshold == params->fThreshold) &&
                (pumping == params->fPumping))
                return false;

            params->pFunc           = func;
            params->fThreshold      = threshold;
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
                dst[i]      = clip_curve(p, x[i]);
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
            sLufs.sGain.set_threshold(dspu::lufs_to_gain(sLufs.pThreshold->value()));

            fStereoLink             = (pStereoLink != NULL) ? pStereoLink->value() * 0.01f : 1.0f;
            nFlags                  = lsp_setflag(nFlags, CF_ODP_ENABLED, sOdp.pOn->value() >= 0.5f);
            if (update_odp_params(&sOdp))
            {
                calc_odp_compressor(&sComp, &sOdp);
                nFlags                 |= CF_SYNC_ODP;
            }
            nFlags                  = lsp_setflag(nFlags, CF_CLIP_ENABLED, sClip.pOn->value() >= 0.5f);
            if (update_clip_params(&sClip))
                nFlags                 |= CF_SYNC_CLIP;

            // Adjust the compensation delays
            const float reactivity      = sOdp.pReactivity->value();
            const size_t latency        = dspu::millis_to_samples(fSampleRate, reactivity) * 0.5f;

            // Compute the final latency
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

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

                c->fClipIn          = GAIN_AMP_M_INF_DB;
                c->fClipOut         = GAIN_AMP_M_INF_DB;
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

                // Measure signal at the input of the band
                const size_t idx_in_l   = dsp::abs_max_index(l->vData, samples);
                const size_t idx_in_r   = dsp::abs_max_index(r->vData, samples);
                const float in_l        = fabsf(l->vData[idx_in_l]);
                const float in_r        = fabsf(r->vData[idx_in_r]);
                l->sInGraph.process(l->vData, samples);
                r->sInGraph.process(r->vData, samples);

                // Overdrive protection
                if (nFlags & CF_ODP_ENABLED)
                {
                    // Measure input signal
                    const size_t odp_idx_l  = dsp::abs_max_index(l->vSc, samples);
                    const size_t odp_idx_r  = dsp::abs_max_index(r->vSc, samples);
                    const float odp_in_l    = l->vSc[odp_idx_l];
                    const float odp_in_r    = r->vSc[odp_idx_r];

                    // Apply ODP
                    odp_gain(l->vSc, l->vSc, &sComp, samples);
                    odp_gain(r->vSc, r->vSc, &sComp, samples);
                    dsp::mul2(l->vData, l->vSc, samples);
                    dsp::mul2(r->vData, r->vSc, samples);

                    // Measure output
                    const float odp_red_l   = l->vSc[odp_idx_l];
                    const float odp_red_r   = r->vSc[odp_idx_r];
                    const float odp_out_l   = odp_in_l * odp_red_l;
                    const float odp_out_r   = odp_in_r * odp_red_r;

                    l->fOdpIn               = lsp_max(l->fOdpIn, odp_in_l);
                    l->fOdpOut              = lsp_max(l->fOdpOut, odp_out_l);
                    l->fOdpRed              = lsp_min(l->fOdpRed, odp_red_l);

                    r->fOdpIn               = lsp_max(r->fOdpIn, odp_in_r);
                    r->fOdpOut              = lsp_max(r->fOdpOut, odp_out_r);
                    r->fOdpRed              = lsp_min(r->fOdpRed, odp_red_r);
                }
                else
                {
                    dsp::fill_one(l->vSc, samples);
                    dsp::fill_one(r->vSc, samples);

                    l->fOdpIn               = GAIN_AMP_M_INF_DB;
                    l->fOdpOut              = GAIN_AMP_M_INF_DB;
                    l->fOdpRed              = GAIN_AMP_0_DB;

                    r->fOdpIn               = GAIN_AMP_M_INF_DB;
                    r->fOdpOut              = GAIN_AMP_M_INF_DB;
                    r->fOdpRed              = GAIN_AMP_0_DB;
                }

                // Clipping
                if (nFlags & CF_CLIP_ENABLED)
                {
                    // Mesure input
                    const size_t clip_idx_l = dsp::abs_max_index(l->vData, samples);
                    const size_t clip_idx_r = dsp::abs_max_index(r->vData, samples);
                    const float clip_in_l   = fabsf(l->vData[clip_idx_l]);
                    const float clip_in_r   = fabsf(r->vData[clip_idx_r]);

                    // Do clipping
                    clip_curve(l->vData, l->vData, &sClip, samples);
                    clip_curve(r->vData, r->vData, &sClip, samples);

                    // Measure output
                    const float clip_out_l  = fabsf(l->vData[clip_idx_l]);
                    const float clip_out_r  = fabsf(r->vData[clip_idx_r]);
                    const float clip_red_l  = (clip_in_l >= GAIN_AMP_M_120_DB) ? clip_out_l / clip_in_l : GAIN_AMP_0_DB;
                    const float clip_red_r  = (clip_in_r >= GAIN_AMP_M_120_DB) ? clip_out_r / clip_in_r : GAIN_AMP_0_DB;

                    l->fClipIn              = lsp_max(l->fClipIn, clip_in_l);
                    l->fClipOut             = lsp_max(l->fClipOut, clip_out_l);
                    l->fClipRed             = lsp_min(l->fClipRed, clip_red_l);

                    r->fClipIn              = lsp_max(r->fClipIn, clip_in_r);
                    r->fClipOut             = lsp_max(r->fClipOut, clip_out_r);
                    r->fClipRed             = lsp_min(r->fClipRed, clip_red_r);
                }
                else
                {
                    l->fClipIn              = GAIN_AMP_M_INF_DB;
                    l->fClipOut             = GAIN_AMP_M_INF_DB;
                    l->fClipRed             = GAIN_AMP_0_DB;

                    r->fClipIn              = GAIN_AMP_M_INF_DB;
                    r->fClipOut             = GAIN_AMP_M_INF_DB;
                    r->fClipRed             = GAIN_AMP_0_DB;
                }

                // Perform output metering
                const float out_l       = fabsf(l->vData[idx_in_l]);
                const float out_r       = fabsf(r->vData[idx_in_r]);
                const float red_l       = (in_l >= GAIN_AMP_M_120_DB) ? out_l / in_l : GAIN_AMP_0_DB;
                const float red_r       = (in_r >= GAIN_AMP_M_120_DB) ? out_r / in_r : GAIN_AMP_0_DB;
                l->sOutGraph.process(l->vData, samples);
                r->sOutGraph.process(r->vData, samples);

                l->fIn                  = lsp_max(l->fIn, in_l);
                l->fOut                 = lsp_max(l->fOut, out_l);
                l->fRed                 = lsp_min(l->fRed, red_l);

                r->fIn                  = lsp_max(r->fIn, in_r);
                r->fOut                 = lsp_max(r->fOut, out_r);
                r->fRed                 = lsp_min(r->fRed, red_r);

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

                // Measure signal at the input of the band
                const size_t idx_in     = dsp::abs_max_index(c->vData, samples);
                const float in          = fabsf(c->vData[idx_in]);
                c->sInGraph.process(c->vData, samples);

                // Overdrive protection
                if (nFlags & CF_ODP_ENABLED)
                {
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
                else
                {
                    dsp::fill_one(c->vSc, samples);

                    c->fOdpIn               = GAIN_AMP_M_INF_DB;
                    c->fOdpOut              = GAIN_AMP_M_INF_DB;
                    c->fOdpRed              = GAIN_AMP_0_DB;
                }

                // Clipping
                if (nFlags & CF_CLIP_ENABLED)
                {
                    // Mesure input
                    const size_t clip_idx   = dsp::abs_max_index(c->vData, samples);
                    const float clip_in     = fabsf(c->vData[clip_idx]);

                    // Do clipping
                    clip_curve(c->vData, c->vData, &sClip, samples);

                    // Measure output
                    const float clip_out    = fabsf(c->vData[clip_idx]);
                    const float clip_red    = (clip_in >= GAIN_AMP_M_120_DB) ? clip_out / clip_in : GAIN_AMP_0_DB;

                    c->fClipIn              = lsp_max(c->fClipIn, clip_in);
                    c->fClipOut             = lsp_max(c->fClipOut, clip_out);
                    c->fClipRed             = lsp_min(c->fClipRed, clip_red);
                }
                else
                {
                    c->fClipIn              = GAIN_AMP_M_INF_DB;
                    c->fClipOut             = GAIN_AMP_M_INF_DB;
                    c->fClipRed             = GAIN_AMP_0_DB;
                }

                // Perform output metering
                const float out         = fabsf(c->vData[idx_in]);
                const float red         = (in >= GAIN_AMP_M_120_DB) ? out / in : GAIN_AMP_0_DB;
                c->sOutGraph.process(c->vData, samples);

                c->fIn                  = lsp_max(c->fIn, in);
                c->fOut                 = lsp_max(c->fOut, out);
                c->fRed                 = lsp_min(c->fRed, red);

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

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = &vChannels[i];

                c->pIn->set_value(c->fIn / fThresh);
                c->pOut->set_value(c->fOut);
                c->pRed->set_value(c->fRed);

                c->pOdpIn->set_value(c->fOdpIn);
                c->pOdpOut->set_value(c->fOdpOut);
                c->pOdpRed->set_value(c->fOdpRed);

                c->pClipIn->set_value(c->fClipIn);
                c->pClipOut->set_value(c->fClipOut);
                c->pClipRed->set_value(c->fClipRed);
            }
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

            // Output data for each channel
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = &vChannels[i];

                // Output oscilloscope graphs for output clipper
                plug::mesh_t *mesh    = c->pTimeMesh->buffer<plug::mesh_t>();
                if ((mesh != NULL) && (mesh->isEmpty()))
                {
                    // Fill time values
                    float *t        = mesh->pvData[0];
                    float *in       = mesh->pvData[1];
                    float *out      = mesh->pvData[2];
                    float *red      = mesh->pvData[3];

                    dsp::copy(&t[2], vTime, meta::clipper::TIME_MESH_POINTS);
                    dsp::copy(&in[2], c->sInGraph.data(), meta::clipper::TIME_MESH_POINTS);
                    dsp::copy(&out[2], c->sOutGraph.data(), meta::clipper::TIME_MESH_POINTS);

                    for (size_t k=2; k<meta::clipper::TIME_MESH_POINTS + 2; ++k)
                        red[k]      = lsp_max(out[k], GAIN_AMP_M_120_DB) / lsp_max(in[k], GAIN_AMP_M_120_DB);

                    // Generate extra points
                    t[0]            = t[2] + meta::clipper::TIME_HISTORY_GAP;
                    t[1]            = t[0];
                    in[0]           = 0.0f;
                    in[1]           = in[2];
                    out[0]          = out[2];
                    out[1]          = out[2];
                    red[0]          = red[2];
                    red[1]          = red[2];

                    t              += meta::clipper::TIME_MESH_POINTS + 2;
                    in             += meta::clipper::TIME_MESH_POINTS + 2;
                    out            += meta::clipper::TIME_MESH_POINTS + 2;
                    red            += meta::clipper::TIME_MESH_POINTS + 2;

                    t[0]            = t[-1] - meta::clipper::TIME_HISTORY_GAP;
                    t[1]            = t[0];
                    in[0]           = in[-1];
                    in[1]           = 0.0f;
                    out[0]          = out[-1];
                    out[1]          = out[-1];
                    red[0]          = red[-1];
                    red[1]          = red[-1];

                    // Notify mesh contains data
                    mesh->data(4, meta::clipper::TIME_MESH_POINTS + 4);
                }
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
            return false;
        }

        void clipper::dump(dspu::IStateDumper *v) const
        {
            plug::Module::dump(v);
        }

    } /* namespace plugins */
} /* namespace lsp */


