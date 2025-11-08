classdef distance_process
    %FRAME_PROCESS Summary of this class goes here
    %   Detailed explanation goes here

    properties
        fft_size
        num_samples
        fft_window
        array_offset_fsk
        array_bin_speed_mps
        max_dist
    end

    methods
        function obj = distance_process(varargin)
            %FRAME_PROCESS Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'fft_size', 64);
            addParameter(p, 'num_samples', 64);
            addParameter(p, 'pulse_time_delta_s', 130e-6);
            addParameter(p, 'offset_static_fsk', 0.16);
            addParameter(p, 'pulse_repetition_time_s', 500e-6);
            addParameter(p, 'center_freq_hz', 24e9);
            addParameter(p, 'max_dist', 3);

            parse(p,varargin{:});
            params = p.Results;
            assert(params.fft_size >= params.num_samples);
            obj.num_samples = double(params.num_samples);
            obj.fft_size = double(params.fft_size);
            obj.fft_window = chebwin(obj.num_samples,60)';

            %% parameters setup
            c0 = physconst('LightSpeed');
            %% calculate derived parameters
            fS = 1/params.pulse_repetition_time_s;          % Sampling frequency
            lambda = c0 / params.center_freq_hz;        % Wavelength
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

        function [output] = run(obj, frame_data)
            %RUN Summary of this method goes here
            %   Detailed explanation goes here
            %% for one antenna BFSK
            % distance, velocity, power
            output = zeros(3,1);
            frame_windowed = (frame_data-mean(frame_data,2)).*obj.fft_window;
            doppler_data = fftshift(fft(frame_windowed, obj.fft_size, 2)/obj.num_samples, 2);
            [ doppler_level, max_idx] = max(abs(doppler_data.'));

            doppler_level_dB = 20*log10(doppler_level);

            %% target estimations
            target_velocity = obj.array_bin_speed_mps(max_idx(1));

            %% target distance estimation
            phase_diff = obj.get_phase_delta(doppler_data(2, :), doppler_data(1, :));
            target_distance_temp = phase_diff * obj.max_dist;
            target_distance_compensated = target_distance_temp + obj.array_offset_fsk;

            %% phase wrapping correction
            target_distance_compensated(target_distance_compensated < 0) = ...
                target_distance_compensated(target_distance_compensated < 0) + obj.max_dist;
            target_distance_compensated(target_distance_compensated > obj.max_dist) = ...
                target_distance_compensated(target_distance_compensated > obj.max_dist) - obj.max_dist;

            % Discard too large ranges, which could be detected due to wrapping for very close targets.
            % (Too slow AOC/AGC can lead to clipping here, which leads to wrong phase estimations.)
            target_distance_compensated(target_distance_compensated > 0.833 * obj.max_dist) = 0;

            output(1) = target_distance_compensated(max_idx(1));
            output(2) = target_velocity;
            output(3) = min(doppler_level_dB);
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
    end

end
