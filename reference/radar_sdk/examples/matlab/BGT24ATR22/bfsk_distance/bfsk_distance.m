% ===========================================================================
% Copyright (C) 2021-2023 Infineon Technologies AG
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
Dev.set_config('BGT24ATR22_settings_bfsk_demo.json');
center_freq_hz = double(Dev.config.AFC_Config.rf_center_frequency_Hz);
num_samples = double(Dev.config.FrameConfig{1}.num_samples);
pulse_repetition_time_s = double(Dev.config.FrameConfig{1}.pulse_repetition_time_s);

max_dist = 1.2; % maximum unambiguity range in meters
power_threshold = -55; %dBFS

%derived parameters
c0 = physconst('LightSpeed');
v_max = c0/(4*center_freq_hz*pulse_repetition_time_s);

%##########################################################################
%% STEP 3:  Compute DUT offset for required unambiguity range
%##########################################################################
[offset_value, max_dist] = Dev.get_dac_offset(max_dist);

Dev.squeeze_pulse_timings();

% configure the bfsk frequency offset for the required pulses
Dev.set_register('VCO_PC0_DAC_OFFSET',offset_value);
Dev.set_register('VCO_PC1_DAC_OFFSET',0);
Dev.set_register('VCO_PC2_DAC_OFFSET',offset_value);
Dev.set_register('VCO_PC3_DAC_OFFSET',0);

MimoseConfig.printconfig(Dev.config);
%##########################################################################
%% STEP 4:  Modify chip configuration
%##########################################################################
%% Squeeze pulses of the configured PCs together, the PRT remains the same
Dev.squeeze_pulse_timings();

%% enable AGC for ABB
Dev.enableAGC();

%% initialize plot object
plot_obj = demo_gui('max_range_1',max_dist,'max_range_2',max_dist, 'v_max', v_max);
process_obj = distance_process('fft_size', num_samples*4, ...
    'num_samples', num_samples, ...
    'pulse_repetition_time_s', pulse_repetition_time_s, ...
    'center_freq_hz', center_freq_hz, ...
    'max_dist', max_dist);
data = zeros(4,0);
fprintf('Close demo plotter to exit program...\n');

%##########################################################################
%% STEP 5: radar data acquisition
%##########################################################################
while ishandle(plot_obj.hfig)

    [last_frame, abb_gains] = Dev.get_next_frame();
    if (isempty(last_frame))
        continue;
    end

    output_antenna1 = process_obj.run(last_frame(1:2,:));
    target_distance_a = output_antenna1(1);
    target_velocity_a = output_antenna1(2);
    target_power_a = output_antenna1(3) + (7-double(abb_gains(1)))*6.02;

    output_antenna2 = process_obj.run(last_frame(3:4,:));
    target_distance_b = output_antenna2(1);
    target_velocity_b = output_antenna2(2);
    target_power_b = output_antenna2(3) + (7-double(abb_gains(3)))*6.02;

    %% refresh data for plotting
    data = [ target_distance_a, target_distance_b, ...
             target_power_a,  target_power_b, ...
             target_velocity_a, target_velocity_b];
    plot_obj.update_plot(data, power_threshold);

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
