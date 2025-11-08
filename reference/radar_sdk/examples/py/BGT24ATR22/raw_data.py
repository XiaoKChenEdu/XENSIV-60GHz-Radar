# ===========================================================================
# Copyright (C) 2023 Infineon Technologies AG
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ===========================================================================

import numpy as np

from ifxradarsdk import get_version_full
import ifxradarsdk.mimose as mimose
import ifxradarsdk.mimose.types as mimose_types
from mimose_plotter import MimosePlotter


print("Radar SDK Version: " + get_version_full())

# open device: The device will be closed at the end of the block. Instead of
# the with-block you can also use:
with mimose.DeviceMimose() as Dev:
    # modify default configuration retrieved from SDK
    Dev.config.FrameConfig[0].selected_pulse_configs[1] = True
    #Dev.config.FrameConfig[0].selected_pulse_configs[2] = True
    Dev.config.PulseConfig[1].channel = mimose_types.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX1_RX2

    # transfer configuration to device
    Dev.set_config()

    # create plot object
    mimose_plot = MimosePlotter(Dev.config)
    fcount = 1

    while 1:
        frame = Dev.get_next_frame()[0]
        mimose_plot.draw(frame)
        if not mimose_plot.is_open():
            break
        print('Got frame ', fcount, ',  num pulse configs=', np.shape(frame)[1])
        fcount = fcount + 1

    mimose_plot.close()
