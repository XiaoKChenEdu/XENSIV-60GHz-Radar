% ===========================================================================
% Copyright (C) 2021 Infineon Technologies AG
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

clear all;

% load package if this is not matlab
if((exist ("OCTAVE_VERSION", "builtin") > 0))
    pkg load communications
end

disp(['Radar SDK Version: ' RadarDevice.get_version_full()]);

%##########################################################################
% STEP 1: create one DeviceConfig instance. Please refer to DeviceConfig.m
% for more details about TR13C or ATR24C (MIMO) configuration. The
% following example configs are also provided.
%##########################################################################



%##########################################################################
% STEP 1: Create and connect to RadarDevice object
%##########################################################################
Dev = RadarDevice();

%##########################################################################
% STEP 2: create one DeviceConfig instance.
% Please refer to DeviceConfig.m for more details about TR13C or
% ATR24C (MIMO) configuration. The following examples of configs are also
% provided.
%##########################################################################

% The following command creates a DeviceConfig object with all params as default values
%   oDevConf = DeviceConfig.mkDevConf();

% For time-domain-multiplexed MIMO: 'mimo_mode': 1. See also DeviceConfig.m.
%   oDevConf = DeviceConfig.mkDevConf('mimo_mode',1,'rx_mask',15);

% Following command creates a DeviceConfig object with frame reptition time of
% 0.5s (and thus a frame rate of 2Hz) and chirp repetition time of 0.5ms:
%    oDevConf = DeviceConfig.mkDevConf('frame_repetition_time_s',0.5, 'chirp_repetition_time_s',0.0005);

% Following command creates a DeviceConfig object with all params as default values

oDevConf = DeviceConfig.mkDevConf();

%##########################################################################
% STEP 3:  Configure the Radar device using the DeviceConfig object created
% in the last STEP. If the configuration can not be set, the device does
% not support the requested combination of RX antennas, i.e. has one RX.
%##########################################################################
try
    Dev.set_config(oDevConf);
catch ME
    disp(ME.message)
    try
    Dev.delete();
    catch ME
        disp(ME.message)
    end
    clear all;
    return
end

%##########################################################################
% STEP 4: Create a ConstantWaveControl object using the function 
% from the connected RadarDevice object.
%##########################################################################
CwControl = Dev.create_cw_control();

%##########################################################################
% STEP 5: Set desired values for configuring the ConstantWave mode. 
%##########################################################################
CwControl.set_num_of_samples_per_antenna(32);
CwControl.set_frequency(60E9);
CwControl.set_tx_dac_value(25);

CwControl.enable_tx_antenna(0, 1);
CwControl.enable_rx_antenna(0, 1);

oBasebandConf = BasebandConfig.mkBasebandConf('vga_gain', ...
    DeviceConfigOptions.ifx_Avian_Baseband_Vga_Gain.IFX_VGA_GAIN_20dB);
CwControl.set_baseband_params(oBasebandConf);

ADC_Conf = AdcConfig.mkAdcConf();
CwControl.set_adc_config(ADC_Conf);

TestSignal_Conf = TestSignalGeneratorConfig.mkTestSignalGeneratorConf('mode', ...
    DeviceConfigOptions.ifx_Avian_Test_Signal_Generator_Mode.IFX_TEST_SIGNAL_MODE_BASEBAND_TEST);
CwControl.set_test_signal_generator_config(TestSignal_Conf);
%##########################################################################
% STEP 6: Enable TX and RX antennas of the Device. 
%##########################################################################
CwControl.enable_tx_antenna(0, 1);
disp("Enabled TX:");
disp(CwControl.is_tx_antenna_enabled(0));

CwControl.enable_rx_antenna(0, 1);
CwControl.enable_rx_antenna(1, 1);
CwControl.enable_rx_antenna(2, 1);
disp("Enabled RX:");
disp(CwControl.is_rx_antenna_enabled(0));
disp(CwControl.is_rx_antenna_enabled(1));
disp(CwControl.is_rx_antenna_enabled(2));

%##########################################################################
% STEP 7: Start signal of continuous wave.
%##########################################################################
CwControl.start_signal();

% Look up the parameters and measurements of the device in CW Mode.
disp("Frequency:");
disp(CwControl.get_frequency());

disp("Dac value:");
disp(CwControl.get_tx_dac_value());

disp("Number of samples:");
samples_per_antenna = double(CwControl.get_num_of_samples_per_antenna());
disp(samples_per_antenna);

disp("Temperature:");
disp(CwControl.measure_temperature());

disp("TX Power 0:");
disp(CwControl.measure_tx_power(0));

%##########################################################################
% STEP 8: Fetch and plot raw data as example.
%##########################################################################
%Fetch frame data and plot the first chirp data of each antenna
ant_num = length(find(de2bi(oDevConf.rx_mask)));
num_ch = ant_num;
if (oDevConf.mimo_mode == 1)
    num_ch = ant_num * 2;
end

fcount = 1;
hFig = figure;
while ishandle(hFig)
    disp(['getting frame ' num2str(fcount)])
    % Fetch next frame data from the CwControl
    last_frame = CwControl.capture_frame();
    if (isempty(last_frame))
        continue;
    end
    if fcount == 1
        plot_axis_y_lo = floor(min(last_frame(:)) * 100) / 100;
        plot_axis_y_hi = ceil(max(last_frame(:)) * 100) / 100;
    end
    % Plot the data coming from each antenna
    plot_axis_y_lo = min(floor(min(last_frame(:)) * 100) / 100, plot_axis_y_lo);
    plot_axis_y_hi = max(ceil(max(last_frame(:)) * 100) / 100, plot_axis_y_hi);
    for i = 1:num_ch
        subplot(1,num_ch, i);
        signal = last_frame(:, i);
        plot(signal);
        % stabilize plot axes
        axis([1 samples_per_antenna plot_axis_y_lo plot_axis_y_hi]);
        xlabel(sprintf('Ant_%d', i - 1));
    end
    drawnow
    fcount = fcount + 1;
end

%##########################################################################
% STEP 9: Stop the signal of the continuous wave.
%##########################################################################
CwControl.stop_signal();

%##########################################################################
% STEP 10: Destroy CW Control and Device in proper order (Option)
%##########################################################################
try
    CwControl.delete();
catch ME
    disp(ME.message)
end

try
    Dev.delete();
catch ME
    disp(ME.message)
end

%##########################################################################
% STEP 11: Destroy all objects in proper order.
%##########################################################################

clear all;

