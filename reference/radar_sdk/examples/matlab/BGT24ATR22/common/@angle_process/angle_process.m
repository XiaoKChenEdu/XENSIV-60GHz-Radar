classdef angle_process
    %DISTANCE_PROCESS class which provides distance information per doppler
    %bin
    %   Detailed explanation goes here

    properties
        config
        lambda
        antenna_spacing
        array_offset_fsk
        array_bin_speed_mps
    end

    methods
        function obj = angle_process(varargin)
            %ANGLE_PROCESS Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'config', MimoseConfig());
            addParameter(p, 'fft_size', 64);
            addParameter(p, 'pulse_time_delta_s', 130e-6);
            addParameter(p, 'offset_static_fsk', 0);
            addParameter(p, 'antenna_spacing', 6.22e-3);

            parse(p,varargin{:});
            params = p.Results;

            obj.config = params.config;
            obj.antenna_spacing = params.antenna_spacing;

            assert(params.fft_size >= obj.config.FrameConfig{1}.num_samples);

            %% parameters setup
            c0 = physconst('LightSpeed');
            %% calculate derived parameters
            fS = 1/double(obj.config.FrameConfig{1}.pulse_repetition_time_s);   % Sampling frequency
            obj.lambda = c0 / double(obj.config.AFC_Config.rf_center_frequency_Hz); % Wavelength
            Hz_to_mps_constant = obj.lambda / 2;            % Conversion factor from frequency to speed in m/s
            fD_max = fS / 2;                                % Maximum theoretical value of the Doppler frequency
            fD_per_bin = fD_max / (params.fft_size/2);         % Doppler bin size in Hz
            array_bin_frequency = ((1:params.fft_size) - params.fft_size/2 - 1) * fD_per_bin;
            obj.array_bin_speed_mps =  array_bin_frequency * Hz_to_mps_constant; % Vector of speed in m/s
            %% assigning time delays and offset compensations
            array_offset_dynamic_fsk = 2*pi *(-params.pulse_time_delta_s)*array_bin_frequency;
            obj.array_offset_fsk = array_offset_dynamic_fsk - params.offset_static_fsk;
        end

        function [angle_data] = run(obj, doppler_data)
            %RUN Summary of this method goes here
            %   Detailed explanation goes here
                        
            %%  estimation
            phase_diff = angle(doppler_data(2, :))- angle(doppler_data(1, :));
            phase_diff_corrected = phase_diff + obj.array_offset_fsk;

            %% phase wrapping correction
            phase_diff_corrected = obj.phase_wrap(phase_diff_corrected, -pi, pi);

            angle_data = asin(phase_diff_corrected*(obj.lambda/(2*pi*obj.antenna_spacing)));
        end

    end

    methods (Static)
        function [output] = phase_wrap(input, minimum, maximum)
            %  Input args:  input: real scalar/vector/array
            %               minimum: lower limit of wrap around
            %               maximum: upper limit of wrap around
            % Output args: output: corrected value/vector/array
            output = input;
            assert(minimum<maximum);
            while(sum(output<minimum))
                output(output<minimum) = output(output<minimum) + (maximum-minimum);
            end
            while(sum(output>maximum))
                output(output>maximum) = output(output>maximum) - (maximum-minimum);
            end            
        end
    end

end
