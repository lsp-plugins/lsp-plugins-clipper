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

#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/plug-fw/plug.h>
#include <private/meta/clipper.h>

namespace lsp
{
    namespace plugins
    {
        /**
         * Base class for the latency compensation delay
         */
        class clipper: public plug::Module
        {
            protected:
                enum mode_t
                {
                    CD_MONO,
                    CD_STEREO,
                    CD_X2_STEREO
                };

                typedef struct channel_t
                {
                    // DSP processing modules
                    dspu::Delay         sLine;              // Delay line
                    dspu::Bypass        sBypass;            // Bypass

                    // Parameters
                    ssize_t             nDelay;             // Actual delay of the signal
                    float               fDryGain;           // Dry gain (unprocessed signal)
                    float               fWetGain;           // Wet gain (processed signal)

                    // Input ports
                    plug::IPort        *pIn;                // Input port
                    plug::IPort        *pOut;               // Output port
                    plug::IPort        *pDelay;             // Delay (in samples)
                    plug::IPort        *pDry;               // Dry control
                    plug::IPort        *pWet;               // Wet control

                    // Output ports
                    plug::IPort        *pOutDelay;          // Output delay time
                    plug::IPort        *pInLevel;           // Input signal level
                    plug::IPort        *pOutLevel;          // Output signal level
                } channel_t;

            protected:
                size_t              nChannels;          // Number of channels
                channel_t          *vChannels;          // Delay channels
                float              *vBuffer;            // Temporary buffer for audio processing

                plug::IPort        *pBypass;            // Bypass
                plug::IPort        *pGainOut;           // Output gain

                uint8_t            *pData;              // Allocated data

            protected:
                void                do_destroy();

            public:
                explicit clipper(const meta::plugin_t *meta);
                clipper (const clipper &) = delete;
                clipper (clipper &&) = delete;
                virtual ~clipper() override;

                clipper & operator = (const clipper &) = delete;
                clipper & operator = (clipper &&) = delete;

                virtual void        init(plug::IWrapper *wrapper, plug::IPort **ports) override;
                virtual void        destroy() override;

            public:
                virtual void        update_sample_rate(long sr) override;
                virtual void        update_settings() override;
                virtual void        process(size_t samples) override;
                virtual void        dump(dspu::IStateDumper *v) const override;
        };

    } /* namespace plugins */
} /* namespace lsp */


#endif /* PRIVATE_PLUGINS_CLIPPER_H_ */

