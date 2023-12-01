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
            sOdp.pResonance         = NULL;
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
            sOdp.pResonance         = trace_port(ports[port_id++]);
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
            const size_t max_odp_delay      = dspu::hz_to_samples(sr, meta::clipper::ODP_RESONANCE_MIN) * 0.5f;
            const size_t samples_per_dot    = dspu::seconds_to_samples(
                sr, meta::clipper::TIME_HISTORY_MAX / meta::clipper::TIME_MESH_POINTS);

            sLufs.sMeter.set_sample_rate(sr);
            sLufs.sGain.set_sample_rate(sr);
            sOutMeter.set_sample_rate(sr);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->sBypass.init(sr);
                c->sDryDelay.init(max_odp_delay);
                c->sScDelay.init(max_odp_delay);
                c->sSc.init(1, 1000.0f / meta::clipper::ODP_RESONANCE_MIN);
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

        void clipper::update_settings()
        {
            const bool bypass       = pBypass->value() >= 0.5f;
            const size_t dither_bits= decode_dithering(pDithering->value());
            fThresh                 = dspu::db_to_gain(-pThresh->value());

            fInGain                 = pGainIn->value();
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
            const size_t latency        = dspu::hz_to_samples(fSampleRate, sOdp.pResonance->value()) * 0.5f;

            // Compute the final latency
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = &vChannels[i];

                c->sBypass.set_bypass(bypass);
                c->sDither.set_bits(dither_bits);
                c->sScDelay.set_delay(latency);
                c->sDryDelay.set_delay(latency);
            }
            set_latency(latency);
        }

        void clipper::process(size_t samples)
        {
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


