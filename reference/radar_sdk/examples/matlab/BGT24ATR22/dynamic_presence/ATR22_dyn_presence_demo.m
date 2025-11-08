% ===========================================================================
% Copyright (C) 2021-2024 Infineon Technologies AG
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
%    this list of conditions and the following disclaimer.
% 2. Redistributions in binary form must reproduce the above copyright
%    notice, this list of conditions and the following disclaimer in the
%    documentation and/or other materials provided with the distribution.
% 3. Neither the name of the copyright holder nor the names of its
%    contributors may be used to endorse or promote products derived from
%    this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
% ===========================================================================

%##########################################################################
% Application parameters
%##########################################################################

Dyn_Thresh_factor = 7;
auto_Threshold = 64;
manual_threshold = 5;
fft_size = 128;

addpath('lib');
addpath('..\common');

if(exist('Dev','var'))
    clear Dev;
end

%##########################################################################
% Check for library dependancies
%##########################################################################
try
    MimoseDevice.check_path();
catch
    ME = MException(['RadarPath:error'], 'ATR22 MexWrapper not found, add path to MimoseSDKMEXWrapper');
    throw(ME);
end
try
    disp(['Radar SDK Version: ' MimoseDevice.get_version_full()]);
catch
    ME = MException(['RadarDevice:error'], 'SDK library not found, add path containing compiled ATR22 Mex file and RDK libraries');
    throw(ME);
end
%##########################################################################
% STEP 1: Create a Mimose device object and connect to an attached device
%##########################################################################
Dev = MimoseDevice();

%##########################################################################
% STEP 2: Modify configuration parameters.
%##########################################################################
% load and set configuration
Dev.set_config('BGT24ATR22_settings_dynpres_demo.json');

%##########################################################################
% STEP 3:  Configure the Radar device using the DeviceConfig object updated
% in the last STEP.
%##########################################################################
configuration = Dev.config;
configuration.ClockConfig.hf_on_time_usec = 10;
Dev.set_config(configuration);
Dev.set_register('RXABB_CONF',0x0003);
MimoseConfig.printconfig(configuration);
%##########################################################################
% STEP 4:  If RC clock is enabled and the returned system clock deviates
% more than the desired clock then the RC look up table can be tuned
%##########################################################################
%Dev.update_rc_lut();

plot_obj = demo_gui('config', configuration, ...
                    'fft_size', fft_size,    ...
                    'manual_threshold', manual_threshold, ...
                    'dyn_thresh_factor', Dyn_Thresh_factor, ...
                    'graph_type', 'dynamic', ... either 'manual', 'dynamic', or 'auto'
                    'auto_threshold', auto_Threshold);

%##########################################################################
% create the Algorithm and plotting objects
%##########################################################################
doppler_obj = doppler_process('config', Dev.config, ...
                    'fft_size', fft_size, ...
                    'manual_threshold', manual_threshold, ...
                    'dyn_thresh_factor', Dyn_Thresh_factor);

while ishandle(plot_obj.hfig)
    % Fetch next frame data from the RadarDevice
    last_frame = Dev.get_next_frame();
    if (isempty(last_frame))
        continue;
    end
    % Do some processing with the obtained frame.
    [doppler_data_valid, doppler_data, valid_indices, doppler_obj] = doppler_obj.run_dynamic_thresh(last_frame);

    plot_obj.update_plot(doppler_data, doppler_data_valid, doppler_obj.dyn_threshold_matrix);
end

%##########################################################################
% STEP 6: Stop the Radar data acquisition trigerred in the last step by the
% function get_next_frame(). Now the device can be reconfigured and
% re-triggered by get_next_frame()
%##########################################################################
Dev.stop_acquisition();
Dev.registers.view_registers('separator',';','to_file',1,'filename','atr22_regs.csv');

%##########################################################################
% STEP 7: Clear the RadarDevice object. It also automatically disconnects
% from the device.
%##########################################################################
clear Dev
