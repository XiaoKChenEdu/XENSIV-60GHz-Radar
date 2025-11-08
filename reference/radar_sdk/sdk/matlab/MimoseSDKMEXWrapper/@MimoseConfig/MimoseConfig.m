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

classdef MimoseConfig
    %MIMOSECONFIG Class definition for configuring a RadarDevice
    % This matlab class creates an object with all parameters to configure the radar device,
    % similar to the device configuration of the radar_sdk. The DeviceConfig object is required
    % as an input to instantiate a RadarDevice object. This class also provides a helper
    % function mkDevConf() to create a DeviceConfig object and accepts param-value pairs in
    % any order. It assumes default values for any parameter that is not specified.

    properties
        % 
        PulseConfig,

        % 
        FrameConfig,

        % 
        AFC_Config,

        % 
        ClockConfig,
    end

    methods
        function obj = MimoseConfig()
            %MIMOSECONFIG Construct an instance of this class
            %   The function instantiates a MimoseConfig object with
            %   default values
            obj.PulseConfig{1} = PulseConfig.mkConfig('channel', MimoseConfigOptions.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX1_RX1, ...
                'tx_power_level', 63);
            obj.PulseConfig{2} = PulseConfig.mkConfig('channel', MimoseConfigOptions.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX2_RX2, ...
                'tx_power_level', 63);
            obj.PulseConfig{3} = PulseConfig.mkConfig('channel', MimoseConfigOptions.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX1_RX2);
            obj.PulseConfig{4} = PulseConfig.mkConfig('channel', MimoseConfigOptions.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX2_RX1);

            obj.FrameConfig{1} = FrameConfig.mkConfig('frame_repetition_time_s', 0.250, ...
                'pulse_repetition_time_s', 0.001, ...
                'selected_pulse_config_0', true, ...
                'num_of_samples', 128);
            obj.FrameConfig{2} = FrameConfig.mkConfig();
            obj.AFC_Config  = AFC_Config.mkConfig;
            obj.ClockConfig = ClockConfig.mkConfig;
        end

        function obj = read_config_file(obj, filename)
            decodedjson = MimoseDevice.readjson(filename);
            if(isempty(decodedjson))
                error('MimoseConfig: configuration file not found');
            end
            % Pulse Configurations
            for i = 1:4
                obj.PulseConfig{i}.abb_gain_type = ...
                    MimoseConfigOptions.ifx_Mimose_ABB_type_t(decodedjson.device_config.pulse_config(i).abb_gain_index);
                obj.PulseConfig{i}.aoc_mode = ...
                    MimoseConfigOptions.ifx_Mimose_AOC_Mode_t(decodedjson.device_config.pulse_config(i).aoc_index);
                obj.PulseConfig{i}.channel = obj.get_channel_type(decodedjson.device_config.pulse_config(i).tx_antennas, ...
                    decodedjson.device_config.pulse_config(i).rx_antennas);
                obj.PulseConfig{i}.tx_power_level = decodedjson.device_config.pulse_config(i).tx_power_level;
            end
            % Frame Configurations
            for i = 1:2
                obj.FrameConfig{i}.frame_repetition_time_s = ...
                    decodedjson.device_config.frame_config(i).frame_repetition_time_s;
                obj.FrameConfig{i}.pulse_repetition_time_s = ...
                    decodedjson.device_config.frame_config(i).pulse_repetition_time_s;
                obj.FrameConfig{i}.selected_pulse_config_0 = ...
                    sum((decodedjson.device_config.frame_config(i).selected_pulse_configs==1));
                obj.FrameConfig{i}.selected_pulse_config_1 = ...
                    sum((decodedjson.device_config.frame_config(i).selected_pulse_configs==2));
                obj.FrameConfig{i}.selected_pulse_config_2 = ...
                    sum((decodedjson.device_config.frame_config(i).selected_pulse_configs==3));
                obj.FrameConfig{i}.selected_pulse_config_3 = ...
                    sum((decodedjson.device_config.frame_config(i).selected_pulse_configs==4));
                obj.FrameConfig{i}.num_samples = decodedjson.device_config.frame_config(i).num_samples;
            end
            % AFC Configuration
            obj.AFC_Config.band = ...
                MimoseConfigOptions.ifx_Mimose_RF_Band_t(decodedjson.device_config.afc_config.band_index);
            obj.AFC_Config.rf_center_frequency_Hz = ...
                decodedjson.device_config.afc_config.rf_center_frequency_hz;
            obj.AFC_Config.afc_duration_ct = ...
                decodedjson.device_config.afc_config.afc_duration_ct;
            obj.AFC_Config.afc_threshold_course = ...
                decodedjson.device_config.afc_config.afc_threshold_course;
            obj.AFC_Config.afc_threshold_fine = ...
                decodedjson.device_config.afc_config.afc_threshold_fine;
            obj.AFC_Config.afc_period = ...
                decodedjson.device_config.afc_config.afc_period;
            obj.AFC_Config.afc_repeat_count = ...
                MimoseConfigOptions.ifx_Mimose_AFC_Repeat_Count_t(decodedjson.device_config.afc_config.afc_repeat_count_index);
            % Clock configuration
            obj.ClockConfig.reference_clock_Hz = decodedjson.device_config.clock_config.reference_clock_Hz;
            obj.ClockConfig.system_clock_Hz = decodedjson.device_config.clock_config.clock_frequency_hz;
            obj.ClockConfig.rc_clock_enabled = decodedjson.device_config.clock_config.clock_enabled;
            obj.ClockConfig.hf_on_time_usec = decodedjson.device_config.clock_config.hf_on_time_usec;
            obj.ClockConfig.system_clock_divider = decodedjson.device_config.clock_config.system_clock_divider;
            obj.ClockConfig.system_clock_div_flex = decodedjson.device_config.clock_config.system_clock_div_flex;
            obj.ClockConfig.sys_clk_to_i2c = decodedjson.device_config.clock_config.sys_clk_to_i2c;
        end
    end

    methods (Static)
        function printconfig(config)
            disp(config.PulseConfig{1});
            disp(config.PulseConfig{2});
            disp(config.PulseConfig{3});
            disp(config.PulseConfig{4});
            disp(config.FrameConfig{1});
            disp(config.FrameConfig{2});
            disp(config.AFC_Config);
            disp(config.ClockConfig);
        end
    end

    methods (Static, Hidden)
        function channel_type = get_channel_type(tx_antenna, rx_antenna)
            if(tx_antenna == 1)
                if(rx_antenna == 1) %TX1RX1
                    channel_type = MimoseConfigOptions.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX1_RX1;
                else %TX1RX2
                    channel_type = MimoseConfigOptions.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX1_RX2;
                end
            else
                if(rx_antenna == 1) %TX2RX1
                    channel_type = MimoseConfigOptions.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX2_RX1;
                else %TX2RX2
                    channel_type = MimoseConfigOptions.ifx_Mimose_Channel_t.IFX_MIMOSE_CHANNEL_TX2_RX2;
                end
            end
        end
    end

end

