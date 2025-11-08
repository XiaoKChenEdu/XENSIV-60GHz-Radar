classdef kick_detection
    %KICK_DETECTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        filter_length
        power_threshold_rear
        power_threshold_down
        pulse1_state
        pulse2_state
        signal_history
        index
        debug
        features
        feature_limits
    end
    
    methods
        function obj = kick_detection(varargin)
            %KICK_DETECTION Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'filter_length', 5);
            addParameter(p, 'power_threshold_rear', 300);
            addParameter(p, 'power_threshold_down', 200);
            addParameter(p, 'debug', 0);

            parse(p,varargin{:});
            params = p.Results;
            obj.filter_length = params.filter_length;
            obj.power_threshold_rear = params.power_threshold_rear;
            obj.power_threshold_down = params.power_threshold_down;
            obj.pulse1_state = pulse_state_object();
            obj.pulse2_state = pulse_state_object();
            obj.signal_history = [ zeros(params.filter_length, 2) ones(params.filter_length, 2)*-60];
            obj.index = 1;
            obj.debug = params.debug;
            obj.features = zeros(20,1);
            obj.feature_limits = zeros(20,2);
            obj.feature_limits(1,:)   = [2.5 3.5]; % obj.pulse1_state.state
            obj.feature_limits(2,:)   = [0 2];     % abs(obj.pulse1_state.negative_start_fcount - obj.pulse2_state.negative_start_fcount);
            obj.feature_limits(3,:)   = [4 20];    % obj.pulse2_state.negative_start_fcount;
            obj.feature_limits(4,:)   = [10 44];   % obj.pulse2_state.positive_start_fcount;
            obj.feature_limits(5,:)   = [-2 58];    % obj.pulse1_state.positive_start_fcount - obj.pulse2_state.positive_start_fcount;
            obj.feature_limits(6,:)   = [10 39];   % obj.pulse1_state.max_pos_velocity_fcount;
            obj.feature_limits(7,:)   = [-4 12];    % obj.pulse1_state.max_pos_velocity_fcount - obj.pulse2_state.max_pos_velocity_fcount;
            obj.feature_limits(8,:)   = [1 18];    % obj.pulse2_state.max_neg_velocity_fcount;
            obj.feature_limits(9,:)   = [-9 4];    % obj.pulse1_state.max_neg_velocity_fcount - obj.pulse2_state.max_neg_velocity_fcount;
            obj.feature_limits(10,:)  = [6 18];    % obj.pulse2_state.max_pos_velocity_fcount - obj.pulse2_state.max_neg_velocity_fcount;
            obj.feature_limits(11,:)  = [0 23];    % obj.pulse1_state.max_power_fcount;
            obj.feature_limits(12,:)  = [-12 12];    % obj.pulse1_state.max_power_fcount - obj.pulse2_state.max_power_fcount;
            obj.feature_limits(13,:)  = [4 27];    % obj.pulse1_state.max_positive_v;
            obj.feature_limits(14,:)  = [0 15];    % abs(abs(obj.pulse1_state.max_positive_v) - abs(obj.pulse1_state.max_negative_v));
            obj.feature_limits(15,:)  = [6 35];   % obj.pulse2_state.max_positive_v;
            obj.feature_limits(16,:)  = [0 9.5];     % abs(abs(obj.pulse2_state.max_positive_v) - abs(obj.pulse2_state.max_negative_v));
            obj.feature_limits(17,:)  = [53 85];   % 20*log10(obj.pulse1_state.max_power);
            obj.feature_limits(18,:)  = [-12 15];   % 20*log10(obj.pulse2_state.max_power) - 20*log10(obj.pulse1_state.max_power);
            obj.feature_limits(19,:)  = [60 120];  % 20*log10(obj.pulse1_state.aggregate_power);
            obj.feature_limits(20,:)  = [-16 10];  % 20*log10(obj.pulse2_state.aggregate_power) - 20*log10(obj.pulse1_state.aggregate_power);
        end
        
        function [obj, kick_detected, trig] = run(obj, new_data)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            kick_detected = false;
            obj.signal_history(obj.index, :) = new_data;
            obj = obj.update_index();

            %% filter out
            filtered_signals = median(obj.signal_history,1);
            velocity_pulse1 = filtered_signals(1);
            velocity_pulse2 = filtered_signals(2);
            power_pulse1 = filtered_signals(3);
            power_pulse2 = filtered_signals(4);

            if(power_pulse1 < obj.power_threshold_rear)
                velocity_pulse1 = 0;
            end
            if(power_pulse2 < obj.power_threshold_down)
                velocity_pulse2 = 0;
            end

            [~, obj.pulse1_state] = obj.pulse1_state.update(velocity_pulse1, power_pulse1);
            [trig, obj.pulse2_state] = obj.pulse2_state.update(velocity_pulse2, power_pulse2);
            if(trig)
                [kick_detected, obj] = obj.detection_process();
            end
        end

        function [obj] = update_index(obj)
            obj.index = obj.index+1;
            if(obj.index>obj.filter_length)
                obj.index = 1;
            end
        end

        function [detected, obj] = detection_process(obj)
            obj.features(1) = obj.pulse1_state.state;
            obj.features(2) = abs(obj.pulse1_state.negative_start_fcount - obj.pulse2_state.negative_start_fcount);
            obj.features(3) = obj.pulse2_state.negative_start_fcount;
            obj.features(4) = obj.pulse2_state.positive_start_fcount;
            obj.features(5) = obj.pulse1_state.positive_start_fcount - obj.pulse2_state.positive_start_fcount;
            obj.features(6) = obj.pulse1_state.max_pos_velocity_fcount;
            obj.features(7) = obj.pulse1_state.max_pos_velocity_fcount - obj.pulse2_state.max_pos_velocity_fcount;
            obj.features(8) = obj.pulse2_state.max_neg_velocity_fcount;
            obj.features(9) = obj.pulse1_state.max_neg_velocity_fcount - obj.pulse2_state.max_neg_velocity_fcount;
            obj.features(10) = obj.pulse2_state.max_pos_velocity_fcount - obj.pulse2_state.max_neg_velocity_fcount;
            obj.features(11) = obj.pulse1_state.max_power_fcount;
            obj.features(12) = obj.pulse1_state.max_power_fcount - obj.pulse2_state.max_power_fcount;
            obj.features(13) = obj.pulse1_state.max_positive_v;
            obj.features(14) = abs(abs(obj.pulse1_state.max_positive_v) - abs(obj.pulse1_state.max_negative_v));
            obj.features(15) = obj.pulse2_state.max_positive_v;
            obj.features(16) = abs(abs(obj.pulse2_state.max_positive_v) - abs(obj.pulse2_state.max_negative_v));
            obj.features(17) = 20*log10(obj.pulse1_state.max_power);
            obj.features(18) = 20*log10(obj.pulse2_state.max_power) - 20*log10(obj.pulse1_state.max_power);
            obj.features(19) = 20*log10(obj.pulse1_state.aggregate_power);
            obj.features(20) = 20*log10(obj.pulse2_state.aggregate_power) - 20*log10(obj.pulse1_state.aggregate_power);

            if(obj.debug && (obj.features(1) ==3))
                fprintf(['%d, %d, %d, %d, %d,\n' ...
                         '%d, %d, %d, %d, %d,\n' ...
                         '%d, %d, %.2f, %.2f, %.2f,\n' ...
                         '%.2f, %.1f, %.1f, %.1f, %.1f\n\n'], obj.features);
            end

            result = zeros(size(obj.feature_limits));
            result(:,1) = obj.features>=obj.feature_limits(:,1);
            result(:,2) = obj.features<=obj.feature_limits(:,2);
            detected = sum(result,'all') == numel(result);
        end
    end
end

