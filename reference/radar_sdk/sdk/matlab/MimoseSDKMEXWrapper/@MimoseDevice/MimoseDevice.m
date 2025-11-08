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

classdef MimoseDevice < handle
    %MIMOSEDEVICE Class for an ATR22 device.
    %   An object of this class helps to
    %   - configure a radar device and connect to it
    %   - retrieve radar data from it
    %   - disconnect from it.
    %   This object may be used as a Mimose SDK wrapper in matlab,
    %   as it performs the above tasks similar to Mimose SDK

    properties
        device_handle % mimose device handle
        config
        registers
    end

    properties (Access = private, Hidden)
        active_frame
    end

    methods
        function obj = MimoseDevice(varargin)
            %MIMOSEDEVICE API Constructor for an instance of a Mimose Device.
            % Connects to available ATR22 board or uses the provided
            % recording. The class member 'config' is initialized to the defaults.
            % The device is brought to reset state but is not directly configured.
            % An object of this class helps to
            %   - connect to an attached ATR22 (Mimose) device and configure it
            %   - retrieve radar data from the device.
            %   - disconnect from it.
            %   This object may be used as a Mimose SDK wrapper in matlab,
            %   as it performs the above tasks similar to Mimose SDK
            %
            %   USAGE
            %   Dev = MimoseDevice(); % This creates a Mimose device object
            %   and stores it in variable 'Dev'

            p = inputParser;
            addParameter(p, 'playback_path', '');
            addParameter(p, 'correct_timing', 1);

            parse(p,varargin{:});
            params = p.Results;

            if(isempty(params.playback_path))
                [ec, handle] = DeviceControlM_Mimose('create');
            else
                [ec, handle] = DeviceControlM_Mimose('create_playback_device', params.playback_path, params.correct_timing);
            end
            obj.check_error_code(ec);
            obj.device_handle = handle;
            obj.config = obj.get_config_defaults();
            obj.active_frame = 1;
            obj.registers = MimoseRegisters();
            obj.get_registers();
        end

        function set_config(obj, oDevConf)
            %SET_CONFIG API Configures the board instance with configuration
            % object parameter
            % USAGE
            % Assuming Mimose object 'Dev'
            %
            % Dev.set_config(); % Uses configuration stored in Dev.config
            % and writes this to the attached device.
            %
            % Dev.set_config(new_config); % Uses new configuration
            % object 'new_config' to configure the device. This also
            % updates the Dev.config to new_config.
            %
            % See also MimoseConfig

            % convert all enumerations to integer
            if(nargin < 2)
                config_sdk = obj.convert_configs_mimose_for_sdk(obj.config);
            else
                if(ischar(oDevConf)) % is a filename
                    obj.config = obj.config.read_config_file(oDevConf);
                    config_sdk = obj.convert_configs_mimose_for_sdk(obj.config);
                else
                    config_sdk = obj.convert_configs_mimose_for_sdk(oDevConf);
                end
            end
            ec = DeviceControlM_Mimose('set_config', obj.device_handle, ...
                config_sdk.PulseConfig{1}, ...
                config_sdk.PulseConfig{2}, ...
                config_sdk.PulseConfig{3}, ...
                config_sdk.PulseConfig{4}, ...
                config_sdk.FrameConfig{1}, ...
                config_sdk.FrameConfig{2}, ...
                config_sdk.AFC_Config, ...
                config_sdk.ClockConfig);
            obj.config = obj.get_config();
            obj.get_registers();
            obj.check_error_code(ec);
        end

        function config = get_config(obj)
            %GET_CONFIG API Retrieves configuration from connected device
            %
            % USAGE
            % Assuming Mimose object 'Dev'
            %
            % retrieved_config = Dev.get_config(); % gets configuration
            % from the device and stores in 'retrieved_config'
            %
            % See also MimoseConfig

            config = obj.config;
            [ec, PC1, PC2, PC3, PC4, FC1, FC2, AC, CC] = DeviceControlM_Mimose('get_config', obj.device_handle, ...
                config.PulseConfig{1}, config.FrameConfig{1}, config.AFC_Config, config.ClockConfig);
            obj.check_error_code(ec);
            config.PulseConfig{1} = PC1;
            config.PulseConfig{2} = PC2;
            config.PulseConfig{3} = PC3;
            config.PulseConfig{4} = PC4;
            config.FrameConfig{1} = FC1;
            config.FrameConfig{2} = FC2;
            config.AFC_Config = AC;
            config.ClockConfig = CC;
            config = obj.convert_configs_mimose_from_sdk(config);
        end

        function config = get_config_defaults(obj)
            %GET_CONFIG_DEFAULTS API Returns a Mimose Configuration
            % structure with default configutration values from the SDK.
            %
            % USAGE
            % Assuming Mimose object 'Dev'
            %
            % default_config = Dev.get_config_defaults(); % gets configuration
            % defaults from the SDK and stores in variable of type MimoseConfig 'default_config'
            %
            % See also MimoseConfig

            config = MimoseConfig();
            [ec, PC1, PC2, PC3, PC4, FC1, FC2, AC, CC] = DeviceControlM_Mimose('get_config_defaults', obj.device_handle, ...
                config.PulseConfig{1}, config.FrameConfig{1}, config.AFC_Config, config.ClockConfig);
            obj.check_error_code(ec);
            config.PulseConfig{1} = PC1;
            config.PulseConfig{2} = PC2;
            config.PulseConfig{3} = PC3;
            config.PulseConfig{4} = PC4;
            config.FrameConfig{1} = FC1;
            config.FrameConfig{2} = FC2;
            config.AFC_Config = AC;
            config.ClockConfig = CC;
            config = obj.convert_configs_mimose_from_sdk(config);
        end

        function disconnect(obj)
            %DISCONNECT API disconnect from a Mimose Device attached via USB
            % This method disconnects and frees device from the port where the radar device is connected
            % The device is then made available for connection using other
            % applications
            %
            % Usage
            % Dev.disconnect();

            if obj.device_handle ~= 0
                ec = DeviceControlM_Mimose('destroy', obj.device_handle);
                obj.check_error_code(ec);
                obj.device_handle = 0;
            end
        end

        function [RxFrame, abb_gains] = get_next_frame(obj, timeout_ms_opt)
            %GET_NEXT_FRAME API Method to fetch radar data from Mimose device
            %   This method fetches data from the radar device in terms of one
            %   frame (all samples of all enabled pulse configurations)
            %   and returns a Frame variable. If the acquisition is not
            %   started, then this is also started
            %
            %USAGE
            %  frame = Dev.get_next_frame() % the returned 'frame' is a complex matrix with
            %  dimensions (num_pulses x num_samples)
            RxFrame = [];
            abb_gains = [];
            if exist('timeout_ms_opt','var')
                [ec, cube_dim_1, num_pulses, num_samples, total_samples, abb_gains] = DeviceControlM_Mimose('get_next_frame_timeout', obj.device_handle, timeout_ms_opt);
            else
                [ec, cube_dim_1, num_pulses, num_samples, total_samples, abb_gains] = DeviceControlM_Mimose('get_next_frame', obj.device_handle);
            end
            complex_samples = complex(total_samples(1:2:end), total_samples(2:2:end));
            obj.check_error_code(ec);
            if(cube_dim_1 ~=1)
                ME = MException(['RadarDevice:error'], 'wrong dimension retrieved');
                throw(ME);
            end
            RxFrame = reshape(complex_samples, num_pulses, num_samples);
            abb_gains = MimoseConfigOptions.ifx_Mimose_ABB_type_t(abb_gains);
        end

        function start_acquisition(obj)
            %START_ACQUISITION API starts the acquisition of raw data
            %   This method starts the acquisition of raw data when the radar device is connected
            %
            % USAGE
            % Dev.start_acquisition();
            [ec] = DeviceControlM_Mimose('start_acquisition', obj.device_handle);
            obj.check_error_code(ec);
        end

        function stop_acquisition(obj)
            %STOP_ACQUISITION API stops the acquisition of raw data
            %   This method stops the acquisition of raw data when the radar device is connected
            %
            % USAGE
            % Dev.stop_acquisition();

            [ec] = DeviceControlM_Mimose('stop_acquisition', obj.device_handle);
            obj.check_error_code(ec);
        end

        function set_register(obj, address, value)
            %SET_REGISTER API sets register value on connected mimose device
            %
            %USAGE
            % Dev.set_register(0x0001, 0x0023); % this writes the value
            %                                   % 0x0023 at address 0x001
            %
            % %Named memory locations can also be used
            % Dev.set_register('CLK_CONF',68);
            %
            % see also get_register

            address_numeric = obj.registers.get_address(address);
            if(isnan(address_numeric))
                ME = MException('MimoseWrapper:error', 'invalid address');
                throw(ME);
            end
            address_value_word = bitand(bitshift(uint32(address_numeric),16),0xFFFF0000) + ...
                bitand(uint32(value),0x0000FFFF);

            [ec] = DeviceControlM_Mimose('set_register', obj.device_handle, uint32(address_value_word));
            obj.get_registers(); % update register list
            obj.check_error_code(ec);
        end

        function regval = get_register(obj, address)
            %GET_REGISTER API Gets register value from the connected device
            % This is especially useful for readout values
            %
            %USAGE
            % val = Dev.get_register(0x0001); % retrieves value from
            %                                       % address 0x0001. This
            %                                       % value can also be
            %                                       % expressed in decimal
            %                                       % format
            % val = Dev.get_register('CLK_CONF'); % named registers can
            %                                     % also be used.
            %
            % see also set_register

            address_numeric = obj.registers.get_address(address);
            [err_code, regval] = DeviceControlM_Mimose('get_register', obj.device_handle, address_numeric);
            obj.get_registers(); % update register list
            obj.check_error_code(err_code);
        end

        function update_rc_lut(obj)
            %UPDATE_RC_LUT API updates RC clocking options look up table at
            % in extreme temperatures or sample variations may lead to a
            % closer to the desired RC clock
            %
            %USAGE
            %
            % % before calling set_config
            % Dev.update_rc_lut(); % retunes the RC look up table

            [err_code] = DeviceControlM_Mimose('update_rc_lut', obj.device_handle);
            obj.check_error_code(err_code);
        end

        function fw_info = get_firmware_information(obj)
            %GET_FIRMWARE_INFORMATION API Gets information about the firmware of the connected
            % baseboad to which the Radar shield is attached. returns
            % firmware version as a string
            %
            % USAGE
            % firmware_version_string = Dev.get_firmware_information();

            [err_code, fw_major, fw_minor, fw_build] = DeviceControlM_Mimose('get_firmware_information', obj.device_handle, oFwInfo);
            obj.check_error_code(err_code);
            fw_info = [num2str(fw_major),'.',num2str(fw_minor),'.',num2str(fw_build)];
        end

        function [offset_value, max_dist] = get_dac_offset(obj, max_dist_required)
            %GET_DAC_OFFSET gets the required DAC offset to configure
            % a required maximum unambiguity range. This is done by
            % configuring the RF frequency at a spacing of the required
            % bandwdth from the originally configured one
            % and reading the internal AFC counter.
            % The device is then configured with the originally configured
            % frequency and the internal counter is read again and an
            % offset is calculated. The actual bandwidth, hence actual
            % unambiguity range maybe different and is output from this
            % function. the computed offset may be used in
            % any pulse configuration. The function will result in an error
            % if the computed frequency exceeds the allowed limits.
            % Therefore the originally configured frequency should be set
            % within the allowed band accordingly.
            %
            % USAGE
            % [offset, max_dist] = Dev.get_dac_offset(max_dist_required);
            c0 = physconst('LightSpeed');
            BW = c0 / (2 * max_dist_required);
            center_freq_hz = double(obj.config.AFC_Config.rf_center_frequency_Hz);
            system_clock_hz = double(obj.config.ClockConfig.system_clock_Hz);

            %% Compute DUT offset for required unambiguity range
            obj.config.AFC_Config.rf_center_frequency_Hz = center_freq_hz + BW;
            obj.config.AFC_Config.afc_duration_ct = 320;
            obj.set_config(); %sends the device config to the device
            % start and stop sequencer
            obj.start_acquisition();
            obj.stop_acquisition();
            dac_value_hi = double(obj.get_register('VCO_DAC_VALUE'));
            lo_word = double(obj.get_register('VCO_AFC_CNT0'));
            hi_word = double(obj.get_register('VCO_AFC_CNT1'));
            count_hi_freq = hi_word*2^16 + lo_word;

            obj.config.AFC_Config.rf_center_frequency_Hz = center_freq_hz;
            obj.set_config(); %sends the device config to the device
            % start and stop sequencer
            obj.start_acquisition();
            obj.stop_acquisition();
            dac_value_lo = double(obj.get_register('VCO_DAC_VALUE'));

            lo_word = double(obj.get_register('VCO_AFC_CNT0'));
            hi_word = double(obj.get_register('VCO_AFC_CNT1'));
            count_lo_freq = hi_word*2^16 + lo_word;

            freq_lo = count_lo_freq*system_clock_hz*8/double(obj.config.AFC_Config.afc_duration_ct);
            freq_hi = count_hi_freq*system_clock_hz*8/double(obj.config.AFC_Config.afc_duration_ct);
            freq_delta = freq_hi-freq_lo;

            max_dist = c0 / (2 * freq_delta);
            %% compute offset
            offset_value = dac_value_hi-dac_value_lo;
        end

        function squeeze_pulse_timings(obj)
            system_clock_hz = double(obj.config.ClockConfig.system_clock_Hz);
            pulse_repetition_time_s = obj.config.FrameConfig{1}.pulse_repetition_time_s;
            pulse_time_delta_s = 130e-6; % t0 minimum possible
            % P0<--t0-->P1<--t0-->P2<--t0-->P3<----------t1---------->
            % <---------------------PRT------------------------------>
            active_pulse_configs = obj.get_active_pulse_configs(1);
            if(active_pulse_configs>1)
                remaining_time = pulse_repetition_time_s-(active_pulse_configs-1)*pulse_time_delta_s; % t1
                t0_clock_cycles = round(pulse_time_delta_s*system_clock_hz);
                t0_register_value = obj.time2regval(t0_clock_cycles, 16, 5);
                t1_clock_cycles = round(remaining_time*system_clock_hz);
                t1_register_value = obj.time2regval(t1_clock_cycles, 16, 5);
                t_register_value = t1_register_value;
                if(obj.config.FrameConfig{1}.selected_pulse_config_3)
                    obj.set_register('PC3_CONF_TIME', t_register_value);
                    t_register_value = t0_register_value;
                end
                if(obj.config.FrameConfig{1}.selected_pulse_config_2)
                    obj.set_register('PC2_CONF_TIME', t_register_value);
                    t_register_value = t0_register_value;
                end
                if(obj.config.FrameConfig{1}.selected_pulse_config_1)
                    obj.set_register('PC1_CONF_TIME', t_register_value);
                    t_register_value = t0_register_value;
                end
                if(obj.config.FrameConfig{1}.selected_pulse_config_0)
                    obj.set_register('PC0_CONF_TIME', t_register_value);
                end
            end
        end

        function enableAGC(obj)
            obj.set_register('PC0_AGC', 0x000F);
            obj.set_register('PC1_AGC', 0x000F);
            obj.set_register('PC2_AGC', 0x000F);
            obj.set_register('PC3_AGC', 0x000F);
        end

        function pulse_configs = get_active_pulse_configs(obj, i)
            frame_config = obj.config.FrameConfig{i};
            pulse_configs = double(frame_config.selected_pulse_config_0 + ...
                    frame_config.selected_pulse_config_1 + ...
                    frame_config.selected_pulse_config_2 + ...
                    frame_config.selected_pulse_config_3);
        end

        function delete(obj)
            %DELETE API destroyer for the Mimose Device object
            %
            %USAGE
            % Dev.delete(); %disconnects Mimose object and makes it
            % available for other applications
            obj.disconnect();
        end
    end

    methods(Hidden)
        function [ values ]  = get_registers(obj)
            %GET_REGISTERS get register values from SDK register memory
            [ec, registers] = DeviceControlM_Mimose('get_registers', obj.device_handle);
            obj.check_error_code(ec);
            values = bitand(registers,hex2dec('ffff'));
            obj.registers.set_register_values(values);
        end
    end

    methods(Static, Hidden)
        function config_out = convert_configs_mimose_for_sdk(config_in)
            config_out = config_in;
            % adapt PulseConfig
            for idx = 1:4
                config_out.PulseConfig{idx}.channel = int32(config_in.PulseConfig{idx}.channel);
                config_out.PulseConfig{idx}.abb_gain_type = int32(config_in.PulseConfig{idx}.abb_gain_type);
                config_out.PulseConfig{idx}.aoc_mode = int32(config_in.PulseConfig{idx}.aoc_mode);
            end

            config_out.AFC_Config.band = int32(config_in.AFC_Config.band);
            config_out.AFC_Config.afc_repeat_count = int32(config_in.AFC_Config.afc_repeat_count);
        end

        function config_out = convert_configs_mimose_from_sdk(config_in)
            config_out = config_in;
            % adapt PulseConfig
            for idx = 1:4
                config_out.PulseConfig{idx}.channel = MimoseConfigOptions.ifx_Mimose_Channel_t(config_in.PulseConfig{idx}.channel);
                config_out.PulseConfig{idx}.abb_gain_type = MimoseConfigOptions.ifx_Mimose_ABB_type_t(config_in.PulseConfig{idx}.abb_gain_type);
                config_out.PulseConfig{idx}.aoc_mode = MimoseConfigOptions.ifx_Mimose_AOC_Mode_t(config_in.PulseConfig{idx}.aoc_mode);
            end

            config_out.AFC_Config.band = MimoseConfigOptions.ifx_Mimose_RF_Band_t(config_in.AFC_Config.band);
            config_out.AFC_Config.afc_repeat_count = MimoseConfigOptions.ifx_Mimose_AFC_Repeat_Count_t(config_in.AFC_Config.afc_repeat_count);
        end

        function ret = check_error_code(err_code)
            ret = (err_code ~= 0);
            if(ret)
                description = DeviceControlM_Mimose(':describe_error', err_code);
                ME = MException(['RadarDevice:error' num2str(err_code)], description);
                throw(ME);
            end
        end
    end

    methods(Static)

        function version = get_version_full()
            % Returns the full SDK version string including
            % the git tag from which this release was built
            %
            % USAGE
            % disp(['Radar SDK Version: ' MimoseDevice.get_version_full()]);
            [~, version] = DeviceControlM_Mimose('get_version_full');
        end

        function version = get_version()
            % Returns the short SDK version string (excluding
            % the git tag from which this release was built)
            %
            %USAGE
            % disp(['Radar SDK Version: ' MimoseDevice.get_version()]);
            [~, version] = DeviceControlM_Mimose('get_version');
        end

        function result = check_path()
            % simple check to ensure if wrapper path is added to the
            % application space
            %
            %USAGE
            % try
            %    MimoseDevice.check_path();
            % catch
            %     ME = MException(['RadarPath:error'], 'ATR22 MexWrapper not found, add path to MimoseSDKMEXWrapper');
            %     throw(ME);
            % end
            result = 1;
        end

        function [reg_value] = time2regval(value, reg_bits, exp_bits)
            assert(value>0);
            mul_bits = reg_bits-exp_bits;
            significant_bits = floor(log2(value))+1;
            if(significant_bits > mul_bits)
                exp_value = significant_bits-mul_bits;
                mul_value = round(value/2^(exp_value));
            else
                mul_value = value;
                exp_value = 0;
            end
            reg_value = mul_value*pow2(exp_bits)+exp_value;
        end

        function [decoded_json] = readjson(filename)
            %READJSON Reads json file
            %   Detailed explanation goes here
            decoded_json = [];
            fidjson = fopen(filename,'rt');
            if(fidjson<0)
                warning("Matching filename not found!");
                return;
            end
            jsondata = fread(fidjson);
            fclose(fidjson);
            decoded_json = jsondecode(char(jsondata'));
        end

    end

end

