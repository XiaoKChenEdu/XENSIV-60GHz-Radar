classdef micromotion_process
    %DISTANCE_PROCESS class which provides distance information per doppler
    %bin
    %   Detailed explanation goes here

    properties
        config
        history
        threshold
        count
        last_sample
        sample_indices
        pulse_joining_threshold
        correction_factor
    end

    methods
        function obj = micromotion_process(varargin)
            %ANGLE_PROCESS Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'config', MimoseConfig());
            addParameter(p, 'observation_time_s', 2.5);
            addParameter(p, 'threshold', 120);
            addParameter(p, 'samples_per_pulse', 1);
            addParameter(p, 'pulse_joining_threshold', 450);

            parse(p,varargin{:});
            params = p.Results;

            obj.config = params.config;
            obj.threshold = params.threshold;
            observation_length = round(params.observation_time_s / double(obj.config.FrameConfig{1}.frame_repetition_time_s));

            samples_per_pulse = params.samples_per_pulse;
            obj.pulse_joining_threshold = params.pulse_joining_threshold;
            obj.history = zeros(samples_per_pulse, observation_length);
            obj.count = 0;
            obj.last_sample = 0;
            step = round(obj.config.FrameConfig{1}.num_samples/samples_per_pulse);
            obj.sample_indices = 1:step:obj.config.FrameConfig{1}.num_samples;
            obj.correction_factor = 0;
        end

        function [micromotion_detected, micromotion_level, obj] = run(obj, pulse_data, gain)
            %RUN Summary of this method goes here
            %   Detailed explanation goes here
            if(isempty(find(pulse_data == 0, 1)))
                gain_scale = double(8-gain);
                %%  scale to ADC Value and bring mid point to zero
                pulse_data_scaled = (pulse_data*4095 - 2048)*gain_scale;

                pulse_data_corrected = pulse_data_scaled - obj.correction_factor;

                %% check if correction required per component
                diff_i = real(pulse_data_corrected(1))-real(obj.last_sample);
                if (abs(diff_i) > obj.pulse_joining_threshold)
                    correction_i = diff_i;
                else
                    correction_i = 0;
                end
                diff_q = imag(pulse_data_corrected(1))-imag(obj.last_sample);
                if (abs(diff_q) > obj.pulse_joining_threshold)
                    correction_q = diff_q;
                else
                    correction_q = 0;
                end

                obj.correction_factor = obj.correction_factor + complex(correction_i, correction_q);

                pulse_data = pulse_data_scaled - obj.correction_factor;
                obj.history(:,1:end-1) = obj.history(:,2:end);
                obj.history(:,end) = pulse_data(obj.sample_indices);
                obj.last_sample = pulse_data(end);
                obj.count = obj.count+1;
            end
            if(obj.count>=size(obj.history,2))
                micromotion_level = max(real(obj.history),[],"all") - min(real(obj.history),[],"all") + ...
                    max(imag(obj.history),[],"all") - min(imag(obj.history),[],"all");
                micromotion_detected = micromotion_level>obj.threshold;
            else
                micromotion_level = 0;
                micromotion_detected = false;
            end
        end
    end
end
