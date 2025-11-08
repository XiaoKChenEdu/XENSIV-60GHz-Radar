classdef frame_process
    %FRAME_PROCESS Summary of this class goes here
    %   Detailed explanation goes here

    properties
        fft_size
        num_samples
        fft_window
        zero_padded
    end

    methods
        function obj = frame_process(varargin)
            %FRAME_PROCESS Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'fft_size', 64);
            addParameter(p, 'num_samples', 64);

            parse(p,varargin{:});
            params = p.Results;
            assert(params.fft_size >= params.num_samples);
            obj.num_samples = double(params.num_samples);
            obj.fft_size = double(params.fft_size);
            obj.zero_padded = zeros(params.fft_size,1);
            obj.fft_window = window(@blackmanharris,obj.num_samples)';
        end

        function [output] = run(obj, frame_data)
            %RUN Summary of this method goes here
            %   Detailed explanation goes here
            %% pulse TX1RX1
            output = zeros(4,1);
            mat = frame_data(1, :);
            [output(1), output(3)] = obj.get_pulse_data(mat);

            %% pulse TX2RX2
            if(size(frame_data,1) == 2)
                mat = frame_data(2, :);
                [output(2), output(4)] = obj.get_pulse_data(mat);
            end
        end

        function [velocity, max_value_dB] = get_pulse_data(obj, mat)
            mean_removed = mat-mean(mat);
            windowed_signal = obj.fft_window.*mean_removed;
            x = abs(fft(windowed_signal,obj.fft_size)/obj.num_samples);
            pulse1_fft = fftshift(x);
            [max_value1_lin , index  ] = max(pulse1_fft);
            max_value_dB = 20*log10(max_value1_lin);
            velocity = (index - obj.fft_size/2 -1);
        end
    end
    
end

