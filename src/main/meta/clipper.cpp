/*
 * Copyright (C) 2024 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2024 Vladimir Sadovnikov <sadko4u@gmail.com>
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

#include <lsp-plug.in/plug-fw/meta/ports.h>
#include <lsp-plug.in/shared/meta/developers.h>
#include <private/meta/clipper.h>

#define LSP_PLUGINS_CLIPPER_VERSION_MAJOR       1
#define LSP_PLUGINS_CLIPPER_VERSION_MINOR       0
#define LSP_PLUGINS_CLIPPER_VERSION_MICRO       7

#define LSP_PLUGINS_CLIPPER_VERSION  \
    LSP_MODULE_VERSION( \
        LSP_PLUGINS_CLIPPER_VERSION_MAJOR, \
        LSP_PLUGINS_CLIPPER_VERSION_MINOR, \
        LSP_PLUGINS_CLIPPER_VERSION_MICRO  \
    )

namespace lsp
{
    namespace meta
    {
        static const port_item_t sigmoid_functions[] =
        {
            { "Hard clip",          "sigmoid.hardclip"                      },
            { "Parabolic",          "sigmoid.parabolic"                     },
            { "Sine",               "sigmoid.sine"                          },
            { "Logistic",           "sigmoid.logistic"                      },
            { "Arctangent",         "sigmoid.arctangent"                    },
            { "Hyperbolic tangent", "sigmoid.hyperbolic_tangent"            },
            { "Guidermannian",      "sigmoid.guidermannian"                 },
            { "Error function",     "sigmoid.error_function"                },
            { "Smoothstep",         "sigmoid.smoothstep"                    },
            { "Smootherstep",       "sigmoid.smootherstep"                  },
            { "Circle",             "sigmoid.circle"                        },

            { NULL, NULL }
        };

        static port_item_t clipper_dither_modes[] =
        {
            { "None",               "dither.none"                           },
            { "7bit",               "dither.bits.7"                         },
            { "8bit",               "dither.bits.8"                         },
            { "11bit",              "dither.bits.11"                        },
            { "12bit",              "dither.bits.12"                        },
            { "15bit",              "dither.bits.15"                        },
            { "16bit",              "dither.bits.16"                        },
            { "23bit",              "dither.bits.23"                        },
            { "24bit",              "dither.bits.24"                        },
            { NULL, NULL }
        };

        #define CLIPPER_COMMON \
            BYPASS, \
            IN_GAIN, \
            OUT_GAIN, \
            SWITCH("lufs_on", "Enable input LUFS limitation", 1.0f), \
            CONTROL("lufs_th", "Input LUFS limiter threshold", U_LUFS, clipper::LUFS_THRESH), \
            LUFS_METER("lufs_rl", "Reduction LUFS value", 24.0f), \
            METER_OUT_GAIN("lufs_gr", "Input LUFS gain reduction", GAIN_AMP_0_DB), \
            LUFS_METER("lufs_il", "Input LUFS value", 24.0f), \
            LUFS_METER("lufs_ol", "Output LUFS value", 24.0f), \
            CONTROL("thresh", "Clipping threshold", U_DB, clipper::THRESHOLD), \
            SWITCH("boost", "Boosting mode", 1.0f), \
            COMBO("dither", "Dithering mode", 0, clipper_dither_modes), \
            SWITCH("clog", "Clipper logarithmic display", 1.0f), \
            SWITCH("op", "Overdrive protection", 1.0f), \
            CONTROL("th", "Overdrive protection threshold", U_DB, clipper::ODP_THRESHOLD), \
            CONTROL("kn", "Overdrive protection knee", U_DB, clipper::ODP_KNEE), \
            LOG_CONTROL("or", "Overdrive protection reactivity", U_MSEC, clipper::ODP_REACT), \
            MESH("opc", "Overdrive protection chart", 2, clipper::CURVE_MESH_POINTS), \
            SWITCH("ce", "Clipper enable", 1.0f), \
            COMBO("cf", "Clipper sigmoid function", 2.0f, sigmoid_functions), \
            LOG_CONTROL("ct", "Clipper sigmoid threshold", U_GAIN_AMP, clipper::CLIP_THRESHOLD), \
            CONTROL("cp", "Clipper sigmoid pumping", U_DB, clipper::CLIP_PUMPING), \
            MESH("cfc", "Clipper sigmoid function chart", 4, clipper::CURVE_MESH_POINTS)

        #define CLIPPER_COMMON_STEREO \
            CLIPPER_COMMON, \
            CONTROL_DFL("slink", "Stereo link", U_PERCENT, clipper::STEREO_LINK, 50.0f)

        #define CLIPPER_METERS(id, label) \
            METER_OUT_GAIN("ilm" id, "Input level meter" label, GAIN_AMP_P_36_DB), \
            METER_OUT_GAIN("olm" id, "Output level meter" label, GAIN_AMP_P_36_DB), \
            METER_GAIN_DFL("grm" id, "Gain reduction level meter" label, GAIN_AMP_P_72_DB, GAIN_AMP_0_DB), \
            METER_OUT_GAIN("odx" id, "Overdrive protection input meter" label, GAIN_AMP_P_36_DB), \
            METER_OUT_GAIN("ody" id, "Overdrive protection output meter" label, GAIN_AMP_P_36_DB), \
            METER_GAIN_DFL("odr" id, "Overdrive protection reduction level meter" label, GAIN_AMP_P_72_DB, GAIN_AMP_0_DB), \
            METER_OUT_GAIN("cfx" id, "Clipping function input meter" label, GAIN_AMP_P_36_DB), \
            METER_OUT_GAIN("cfy" id, "Clipping function output meter" label, GAIN_AMP_P_36_DB), \
            METER_GAIN_DFL("cfr" id, "Clipping function reduction level meter" label, GAIN_AMP_P_72_DB, GAIN_AMP_0_DB), \
            MESH("ctg" id, "Clipper time graph" label, 4, clipper::TIME_MESH_POINTS + 4)

        #define OSCILLOSCOPE_SWITCHES(id, label) \
            SWITCH("ilg" id, "Input level graph enable" label, 1.0f), \
            SWITCH("olg" id, "Output level graph enable" label, 1.0f), \
            SWITCH("grg" id, "Gain reduction graph enable" label, 1.0f)

        //-------------------------------------------------------------------------
        // Plugin metadata

        static const port_t clipper_mono_ports[] =
        {
            PORTS_MONO_PLUGIN,
            CLIPPER_COMMON,
            OSCILLOSCOPE_SWITCHES("", ""),
            CLIPPER_METERS("", ""),

            PORTS_END
        };

        static const port_t clipper_stereo_ports[] =
        {
            // Input and output audio ports
            PORTS_STEREO_PLUGIN,
            CLIPPER_COMMON_STEREO,
            OSCILLOSCOPE_SWITCHES("_l", " Left"),
            OSCILLOSCOPE_SWITCHES("_r", " Right"),
            CLIPPER_METERS("_l", " Left"),
            CLIPPER_METERS("_r", " Right"),

            PORTS_END
        };

        static const int plugin_classes[]       = { C_DYNAMICS, -1 };
        static const int clap_features_mono[]   = { CF_AUDIO_EFFECT, CF_MASTERING, CF_MONO, -1 };
        static const int clap_features_stereo[] = { CF_AUDIO_EFFECT, CF_MASTERING, CF_STEREO, -1 };

        const meta::bundle_t clipper_bundle =
        {
            "clipper",
            "Clipper",
            B_DYNAMICS,
            "jbnGSgoVIkg",
            "Peak clipping tool for maximization of output loudness"
        };

        const plugin_t clipper_mono =
        {
            "Clipper Mono",
            "Clipper Mono",
            "Clipper Mono",
            "CL1M",
            &developers::v_sadovnikov,
            "clipper_mono",
            {
                LSP_LV2_URI("clipper_mono"),
                LSP_LV2UI_URI("clipper_mono"),
                "cl1m",
                LSP_VST3_UID("cl1m    cl1m"),
                LSP_VST3UI_UID("cl1m    cl1m"),
                LSP_LADSPA_CLIPPER_BASE + 0,
                LSP_LADSPA_URI("clipper_mono"),
                LSP_CLAP_URI("clipper_mono"),
                LSP_GST_UID("clipper_mono"),
            },
            LSP_PLUGINS_CLIPPER_VERSION,
            plugin_classes,
            clap_features_mono,
            E_DUMP_STATE | E_INLINE_DISPLAY,
            clipper_mono_ports,
            "dynamics/clipper/mono.xml",
            NULL,
            mono_plugin_port_groups,
            &clipper_bundle
        };

        const plugin_t clipper_stereo =
        {
            "Clipper Stereo",
            "Clipper Stereo",
            "Clipper Stereo",
            "CL1S",
            &developers::v_sadovnikov,
            "clipper_stereo",
            {
                LSP_LV2_URI("clipper_stereo"),
                LSP_LV2UI_URI("clipper_stereo"),
                "cl1s",
                LSP_VST3_UID("cl1s    cl1s"),
                LSP_VST3UI_UID("cl1s    cl1s"),
                LSP_LADSPA_CLIPPER_BASE + 1,
                LSP_LADSPA_URI("clipper_stereo"),
                LSP_CLAP_URI("clipper_stereo"),
                LSP_GST_UID("clipper_stereo"),
            },
            LSP_PLUGINS_CLIPPER_VERSION,
            plugin_classes,
            clap_features_stereo,
            E_DUMP_STATE | E_INLINE_DISPLAY,
            clipper_stereo_ports,
            "dynamics/clipper/stereo.xml",
            NULL,
            stereo_plugin_port_groups,
            &clipper_bundle
        };
    } /* namespace meta */
} /* namespace lsp */



