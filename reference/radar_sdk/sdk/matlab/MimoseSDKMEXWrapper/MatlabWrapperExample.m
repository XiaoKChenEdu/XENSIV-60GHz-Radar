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

%% Paramters for ATR22 example
% If the internal RC clock is enabled and the returned system clock deviates
% more than the desired clock then the RC look up table can be tuned
rc_lut_update = 0;

% There are multiple ways to configure the device
% configuration_method = 1 ... configuration using a *.json file exported from the Fusion GUI
% configuration_method = 2 ... modification of default configuration
% configuration_method = 3 ... configuration using a local configuration variable
% configuration_method = 4 ... read configuration from device, and modify
configuration_method = 1;

if(exist('Dev','var'))
    clear Dev;
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
% STEP 2: Device configuration.
%##########################################################################

%##########################################################################
% If the internal RC clock is enabled and the returned system clock deviates
% more than the desired clock then the RC look up table can be tuned before
% the configuration
%##########################################################################
if (rc_lut_update)
    Dev.update_rc_lut();
end

switch (configuration_method)
    case 1
        %% method 2: Using configuration exported from the Fusion GUI
        Dev.set_config('BGT24ATR22_example_settings_FusionGUI.json');
    case 2
        %% method 2: Modification of default configuration
        Dev.config.FrameConfig{1}.num_samples = 128;
        Dev.config.FrameConfig{1}.selected_pulse_config_1 = true;
        Dev.config.ClockConfig.system_clock_Hz = 10000000;
        Dev.config.ClockConfig.rc_clock_enabled = 1;
        Dev.set_config();
    case 3
        %% method 3: Configuration using local variable
        newConfig = Dev.get_config_defaults();
        newConfig.FrameConfig{1}.selected_pulse_config_1 = true;
        Dev.set_config(newConfig);
    case 4
        %% method 4: Read configuration from device and modify
        % The device should already be configured via the SDK within the current connection.
        % (i.e. between 'Dev = MimoseDevice()' and before 'Dev.get_config()')
        Dev.set_config(); %configures the default device configuration for this example
        newConfig = Dev.get_config();
        newConfig.FrameConfig{1}.selected_pulse_config_1 = true;
        Dev.set_config(newConfig);
    otherwise
        Dev.set_config(); %configures the default device configuration
end

%% print device configuration
Dev.config.printconfig(Dev.config);

%% create plotter
hfig = figure(1);
set(hfig, 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'BGT24ATR22 Example Data Plot');
active_frame = Dev.config.FrameConfig{1};
active_pulses = active_frame.selected_pulse_config_0 + active_frame.selected_pulse_config_1 + ...
    active_frame.selected_pulse_config_2 + active_frame.selected_pulse_config_3;
tiledlayout(hfig, active_pulses, 1);
tile = nan(active_pulses,1);
plots = cell([active_pulses,1]);
signals = ones(active_frame.num_samples, 2)*2048;
for i = 1:active_pulses
    tile(i) = nexttile;
    plots{i} = plot(signals);
    axis(tile(i), [1 active_frame.num_samples 0 4095]);
    legend(tile(i),'I','Q', 'FontSize',7);
    xlabel(tile(i), sprintf('Pulse %d', i-1));
end

%##########################################################################
% STEP 3: Start fetching and displaying raw data I, Q samples
%##########################################################################
%% Start fetching data
fcount = 1;
disp('Close figure to end script ...');
% In this example, only display datasubset from 10 frames.
while ishandle(hfig)
    % Fetch next frame data from the RadarDevice
    last_frame = Dev.get_next_frame();
    if (isempty(last_frame))
        continue;
    end
    % Display the received data in a plot
    for pulse_num = 1:active_pulses
        plots{pulse_num}(1).YData = real(last_frame(pulse_num,:)*4095);
        plots{pulse_num}(2).YData = imag(last_frame(pulse_num,:)*4095);
    end
    title(tile(1), sprintf('Frame number %d', fcount));
    fcount = fcount + 1;
    drawnow;
end

%##########################################################################
% STEP 4: Stop the Radar data acquisition trigerred in the last step by the
% function get_next_frame(). Now the device can be reconfigured and
% re-triggered by get_next_frame()
%##########################################################################
Dev.stop_acquisition();
%% Save device register list to csv file
Dev.registers.view_registers('separator',';','to_file',1,'filename','atr22_regs.csv');
%##########################################################################
% STEP 5: Clear the RadarDevice object. It also automatically disconnects
% from the device.
%##########################################################################
clear Dev

