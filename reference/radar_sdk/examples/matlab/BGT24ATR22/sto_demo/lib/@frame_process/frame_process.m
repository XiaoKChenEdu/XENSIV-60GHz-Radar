classdef frame_process
    %FRAME_PROCESS Summary of this class goes here
    %   Detailed explanation goes here

    properties
        fft_size
        num_samples
        fft_window
        num_targets
        average_vector_length
        magnitude_average
        velocity_average
        average_vector_index
    end

    methods
        function obj = frame_process(varargin)
            %FRAME_PROCESS Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'fft_size', 64);
            addParameter(p, 'num_samples', 64);
            addParameter(p, 'num_targets', 8);
            addParameter(p, 'average_vector_length', 5);

            parse(p,varargin{:});
            params = p.Results;
            assert(params.fft_size >= params.num_samples);
            obj.num_samples = double(params.num_samples);
            obj.fft_size = double(params.fft_size);
            obj.fft_window = kaiser(obj.num_samples, 6)';
            obj.num_targets = params.num_targets;
            obj.average_vector_length = params.average_vector_length;
            obj.average_vector_index = 1;
            obj.magnitude_average = zeros(2, obj.average_vector_length);
            obj.velocity_average = ones(2, obj.average_vector_length)*obj.fft_size/2;
        end

        function [output, obj] = run(obj, frame_data, abb_gains)
            %RUN Summary of this method goes here
            %   process data from both pulses
            output = zeros(4,1);

            %% Scale time domain doppler data
            mat = frame_data*4095.*pow2(7-abb_gains);

            %% remove mean and apply window
            mean_removed = mat - mean(mat,2);
            windowed = mean_removed.*obj.fft_window;

            %% perform FFT
            fft_out = fftshift(fft(windowed, obj.fft_size, 2)/obj.num_samples,2);

            %% scale and abs for device equivalence
            abs_fft_out = 2*abs(fft_out);

            %% find maximum n values and indices
            [max_mags, max_indices] = maxk(abs_fft_out, obj.num_targets, 2);

            mag_sums = sum(max_mags,2);
            vel_indices = sum((max_mags./mag_sums).*max_indices,2);

            %% average of last n frames
            obj.magnitude_average(:, obj.average_vector_index) = mag_sums;
            obj.velocity_average(:, obj.average_vector_index) = vel_indices;
            obj.average_vector_index = obj.average_vector_index + 1;
            if(obj.average_vector_index > obj.average_vector_length)
                obj.average_vector_index = 1;
            end
            output(1:2) = mean(obj.velocity_average,2) - (obj.fft_size/2+1);
            output(3:4) = mean(obj.magnitude_average,2);
        end
    end
    
end

