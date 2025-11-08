classdef doppler_process
    %DOPPLER_PROCESS Summary of this class goes here
    %   Detailed explanation goes here

    properties
        num_samples
        fft_size        
        fft_window
        dyn_thresh_factor
        dyn_thresh_indices
        manual_thresh
        dyn_threshold_matrix
    end

    methods
        function obj = doppler_process(varargin)
            %DOPPLER_PROCESS Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'config', MimoseConfig());
            addParameter(p, 'fft_size', 64);
            addParameter(p, 'dyn_thresh_half_width', 6);
            addParameter(p, 'dyn_thresh_factor', 7.5);
            addParameter(p, 'manual_threshold', 6);
            addParameter(p, 'fft_window', NaN);

            parse(p,varargin{:});
            params = p.Results;
            obj.num_samples = double(params.config.FrameConfig{1}.num_samples);
            obj.fft_size = params.fft_size;
            assert(params.fft_size >= obj.num_samples);
            if(isnan(params.fft_window))
                obj.fft_window = kaiser(obj.num_samples,6)';
            else
                assert(length(params.fft_window(:)) == obj.num_samples);
                obj.fft_window = params.fft_window(:)';
            end
            % compute dynamic threshold averaging indices
            obj.dyn_thresh_indices = zeros(2*params.dyn_thresh_half_width+1, params.fft_size);
            obj.dyn_thresh_indices(params.dyn_thresh_half_width+1, :) = [ 1 fliplr(2:params.fft_size)];
            for i = 1:params.dyn_thresh_half_width
                obj.dyn_thresh_indices(params.dyn_thresh_half_width+1-i,:) = ...
                    obj.dyn_thresh_indices(params.dyn_thresh_half_width+2-i, :)-1;
                obj.dyn_thresh_indices(params.dyn_thresh_half_width+1+i,:) = ...
                    obj.dyn_thresh_indices(params.dyn_thresh_half_width+i, :)+1;
            end
            obj.dyn_thresh_indices(:,1) = 1;
            obj.dyn_thresh_indices(:,params.fft_size/2+1) = params.fft_size/2+1;
            x = obj.dyn_thresh_indices(:,1:params.fft_size/2);
            i = find(x(end,:)>params.fft_size);
            x(:,i) = repmat(x(:,i(end)+1),1,length(i));
            i = find(x(1,:)<=(params.fft_size/2+1));
            x(:,i(2:end)) = repmat(x(:,i(2)-1),1,length(i(2:end)));
            obj.dyn_thresh_indices(:,1:params.fft_size/2) = x;
            x = obj.dyn_thresh_indices(:,params.fft_size/2+2:end);
            i = find(x(end,:)>params.fft_size/2);
            x(:,i) = repmat(x(:,i(end)+1),1,length(i));
            i = find(x(1,:)<2);
            x(:,i) = repmat(x(:,i(1)-1),1,length(i));
            obj.dyn_thresh_indices(:,params.fft_size/2+2:end) = x;

            obj.dyn_thresh_factor = params.dyn_thresh_factor;
            obj.manual_thresh = params.manual_threshold;
        end

        function [output] = run(obj, frame_data_in)
            %RUN return fft result
            %   Detailed explanation goes here
            % convert to 12-bit ADC samples
            mat = frame_data_in*4095;
            mean_removed = mat-mean(mat,2);
            windowed_signal = obj.fft_window.*mean_removed;
            x = fft(windowed_signal,obj.fft_size,2)/obj.num_samples;
            output = 2*fftshift(x, 2);
        end

        function [fft_out_valid, fft_out, valid_indices, obj] = run_dynamic_thresh(obj, frame_data_in)
            %RUN_DYNAMIC_THRESH return fft result and indices where output exceeds dynamic threshold 
            %   Detailed explanation goes here
            % convert to 12-bit ADC samples
            fft_out = obj.run(frame_data_in);
            abs_fft_out = abs(fft_out);
            obj.dyn_threshold_matrix = zeros(size(abs_fft_out));
            for i = 1:size(abs_fft_out,1)
                pulse_abs = abs_fft_out(i,:);
                obj.dyn_threshold_matrix(i,:) = mean(pulse_abs(obj.dyn_thresh_indices),1)*obj.dyn_thresh_factor;
            end
            fft_out_valid = zeros(size(fft_out));
            valid_indices = find(abs_fft_out>obj.dyn_threshold_matrix);
            fft_out_valid(valid_indices) = fft_out(valid_indices);
        end

        function [fft_out_valid, fft_out, valid_indices] = run_manual_thresh(obj, frame_data_in)
           %RUN_MANUAL_THRESH return fft result and indices where output exceeds manual threshold 
            %   Detailed explanation goes here
            % convert to 12-bit ADC samples
            fft_out = obj.run(frame_data_in);
            abs_fft_out = abs(fft_out);
            threshold_matrix = ones(size(abs_fft_out))*obj.manual_thresh;
            fft_out_valid = zeros(size(fft_out));
            valid_indices = find(abs_fft_out>threshold_matrix);
            fft_out_valid(valid_indices) = fft_out(valid_indices);
        end
    end
end

