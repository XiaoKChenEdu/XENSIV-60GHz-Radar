classdef bfsk_distance_process
    %DISTANCE_PROCESS class which provides distance information per doppler
    %bin
    %   Detailed explanation goes here

    properties
        config
        fft_size
        max_dist
        array_offset_fsk
        array_bin_speed_mps
    end

    methods
        function obj = bfsk_distance_process(varargin)
            %FRAME_PROCESS Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'config', MimoseConfig());
            addParameter(p, 'fft_size', 64);
            addParameter(p, 'pulse_time_delta_s', 130e-6);
            addParameter(p, 'offset_static_fsk', 0.16);
            addParameter(p, 'max_dist', 3);

            parse(p,varargin{:});
            params = p.Results;

            obj.config = params.config;
            obj.fft_size = params.fft_size;

            assert(obj.fft_size >= obj.config.FrameConfig{1}.num_samples);

            %% parameters setup
            c0 = physconst('LightSpeed');
            %% calculate derived parameters
            fS = 1/double(obj.config.FrameConfig{1}.pulse_repetition_time_s);   % Sampling frequency
            lambda = c0 / double(obj.config.AFC_Config.rf_center_frequency_Hz); % Wavelength
            Hz_to_mps_constant = lambda / 2;                % Conversion factor from frequency to speed in m/s
            fD_max = fS / 2;                                % Maximum theoretical value of the Doppler frequency
            fD_per_bin = fD_max / (obj.fft_size/2);         % Doppler bin size in Hz
            array_bin_frequency = ((1:obj.fft_size) - obj.fft_size/2 - 1) * fD_per_bin;
            obj.array_bin_speed_mps =  array_bin_frequency * Hz_to_mps_constant; % Vector of speed in m/s
            obj.max_dist = params.max_dist;
            %% assigning time delays and offset compensations
            array_offset_dynamic_fsk = obj.max_dist *(-params.pulse_time_delta_s)*array_bin_frequency;
            obj.array_offset_fsk = array_offset_dynamic_fsk - params.offset_static_fsk;
        end

        function [distance_data] = run(obj, doppler_data)
            %RUN Summary of this method goes here
            %   Detailed explanation goes here
            %% for one antenna BFSK
            % distance, velocity, power
            
            %% distance estimation
            phase_diff = obj.get_phase_delta(doppler_data(2, :), doppler_data(1, :));
            target_distance_temp = phase_diff * obj.max_dist;
            distance_data = target_distance_temp + obj.array_offset_fsk;

            %% phase wrapping correction
            distance_data = obj.phase_wrap(distance_data, 0, obj.max_dist);
            
            % Discard too large ranges, which could be detected due to wrapping for very close targets.
            % (Too slow AOC/AGC can lead to clipping here, which leads to wrong phase estimations.)
            distance_data(distance_data > 0.85 * obj.max_dist) = 0;

        end

    end

    methods (Static)
        function [output] = get_phase_delta(a, b)
            %  Input args:  a: complex scalar/vector
            %               b: complex scalar/vector
            % Output args: phase_diff: phase difference between c1 & c2 in RAD/(2pi)
            phase_delta = (angle(a)+pi)-(angle(b)+pi);
            if(phase_delta<0)
                phase_delta = phase_delta + 2*pi;
            end
            output = phase_delta/(2*pi); % normalize
        end

        function [output] = phase_wrap(input, minimum, maximum)
            %  Input args:  input: real scalar/vector/array
            %               minimum: lower limit of wrap around
            %               maximum: upper limit of wrap around
            % Output args: output: corrected value/vector/array
            output = input;
            assert(minimum<maximum);
            while(sum(output<minimum))
                output(output<minimum) = output(output<minimum) + maximum;
            end
            while(sum(output>maximum))
                output(output>maximum) = output(output>maximum) - maximum;
            end            
        end
    end

end
