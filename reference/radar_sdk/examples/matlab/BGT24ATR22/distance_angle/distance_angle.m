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

% NOTE: This application is intended for the lambda/2 spacing Rx antenna
% design

addpath('lib');
addpath('..\common');

if(exist('Dev','var'))
    clear Dev;
end

%##########################################################################
%% Check for library dependancies
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
%% STEP 1: Create a Mimose device object and connect to an attached device
%##########################################################################
Dev = MimoseDevice();

%##########################################################################
%% STEP 2: Modify configuration parameters.
%##########################################################################
% load configuration
Dev.set_config('BGT24ATR22_settings_distance_angle_demo.json');
center_freq_hz = double(Dev.config.AFC_Config.rf_center_frequency_Hz);
num_samples = double(Dev.config.FrameConfig{1}.num_samples);
pulse_repetition_time_s = double(Dev.config.FrameConfig{1}.pulse_repetition_time_s);

max_dist = 7; % maximum unambiguity range in meters

%derived parameters
c0 = physconst('LightSpeed');
v_max = c0/(4*center_freq_hz*pulse_repetition_time_s);

%##########################################################################
%% STEP 3:  Compute DUT offset for required unambiguity range
%##########################################################################
[offset_value, max_dist] = Dev.get_dac_offset(max_dist);

% configure the bfsk frequency offset for the required pulses
Dev.set_register('VCO_PC0_DAC_OFFSET',offset_value);
Dev.set_register('VCO_PC1_DAC_OFFSET',0);
Dev.set_register('VCO_PC2_DAC_OFFSET',0);

MimoseConfig.printconfig(Dev.config);
%##########################################################################
%% STEP 4:  Modify chip configuration
%##########################################################################
%% Squeeze pulses of the configured PCs together, the PRT remains the same
Dev.squeeze_pulse_timings();

%% enable AGC for ABB
Dev.enableAGC();

%% initialize plot object
plot_obj = demo_gui_da('max_dist', max_dist);


%% initialize process objects
fft_size = num_samples*2;
doppler_obj = doppler_process('config', Dev.config, ...
                    'fft_size', fft_size, ...
                    'manual_threshold', 7, ...
                    'dyn_thresh_factor', 3);
distance_obj = bfsk_distance_process('config', Dev.config, ...
                    'fft_size', fft_size, ...
                    'max_dist', max_dist);

angle_obj = angle_process('config', Dev.config, ...
                    'fft_size', fft_size, ...
                    'offset_static_fsk', -0.5);

micromotion_obj = micromotion_process('config', Dev.config, ...
                    'observation_time_s', 2.5, ...
                    'threshold', 120);
micromotion_pulse_id = 2;
target_obj = target_process();

fprintf('Close demo plotter to exit program...\n');


%##########################################################################
%% STEP 5: radar data acquisition
%##########################################################################
while ishandle(plot_obj.hfig)

    [last_frame, abb_gains] = Dev.get_next_frame();
    if (isempty(last_frame))
        continue;
    end
    
    [doppler_data_valid, doppler_data, valid_indices] = doppler_obj.run_dynamic_thresh(last_frame);
    [distance_data] = distance_obj.run(doppler_data(1:2,:));
    [angle_data] = angle_obj.run(doppler_data(2:3,:));
    [micromotion_detected, micromotion_level, micromotion_obj] = ...
        micromotion_obj.run(last_frame(micromotion_pulse_id,:), abb_gains(micromotion_pulse_id));
    
    indices1 = find(abs(doppler_data_valid(1,:))>0);
    indices2 = find(abs(doppler_data_valid(2,:))>0);
    indices3 = find(abs(doppler_data_valid(3,:))>0);
    indices = intersect(intersect(indices1,indices2), indices3);
    
    distance_data_valid = distance_data(indices);
    angle_data_valid = angle_data(indices)+pi/2;

    [x, y] = pol2cart(angle_data_valid, distance_data_valid);
    [target_x, target_y, target_obj] = target_obj.run(x, y);
    plot_obj.update_points(x,y);
    plot_obj.update_status(indices, micromotion_detected, micromotion_level);
    plot_obj.update_target(target_x, target_y, micromotion_detected);
        
    drawnow;

end

%##########################################################################
%% STEP 6: Stop the Radar data acquisition trigerred in the last step by the
% function get_next_frame(). Now the device can be reconfigured and
% re-triggered by get_next_frame()
%##########################################################################
Dev.stop_acquisition();
Dev.registers.view_registers('separator',';','to_file',1,'filename','atr22_regs.csv');

%##########################################################################
%% STEP 7: Clear the RadarDevice object. It also automatically disconnects
% from the device.
%##########################################################################
clear Dev
