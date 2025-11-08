classdef kick_detection
    %KICK_DETECTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        filter_length
        power_threshold
        pulse1_state
        pulse2_state
        signal_history
        index
        debug
    end
    
    methods
        function obj = kick_detection(varargin)
            %KICK_DETECTION Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'filter_length', 5);
            addParameter(p, 'power_threshold', -50);
            addParameter(p, 'debug', 0);

            parse(p,varargin{:});
            params = p.Results;
            obj.filter_length = params.filter_length;
            obj.power_threshold = params.power_threshold;
            obj.pulse1_state = pulse_state_object();
            obj.pulse2_state = pulse_state_object();
            obj.signal_history = [ zeros(params.filter_length, 2) ones(params.filter_length, 2)*-60];
            obj.index = 1;
            obj.debug = params.debug;
        end
        
        function [obj, kick_detected] = run(obj, new_data)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            kick_detected = false;
            obj.signal_history(obj.index, :) = new_data;
            obj = obj.update_index();

            %% filter out
            filtered_signals = mean(obj.signal_history,1);
            velocity_pulse1 = filtered_signals(1);
            velocity_pulse2 = filtered_signals(2);
            power_pulse1 = filtered_signals(3);
            power_pulse2 = filtered_signals(4);

            if(power_pulse1 < obj.power_threshold)
                velocity_pulse1 = 0;
            end
            if(power_pulse2 < obj.power_threshold)
                velocity_pulse2 = 0;
            end

            [~, obj.pulse1_state] = obj.pulse1_state.update(velocity_pulse1, power_pulse1);
            [trig, obj.pulse2_state] = obj.pulse2_state.update(velocity_pulse2, power_pulse2);
            if(trig)
                kick_detected = obj.detection_process();
            end
        end

        function [obj] = update_index(obj)
            obj.index = obj.index+1;
            if(obj.index>obj.filter_length)
                obj.index = 1;
            end
        end

        function [detected] = detection_process(obj)
            condition = zeros(11,1);
            condition(1) = obj.pulse1_state.state;
            condition(2) = abs(obj.pulse1_state.negative_start_index - obj.pulse2_state.negative_start_index);
            condition(3) = obj.pulse2_state.negative_start_index;
            condition(4) = obj.pulse2_state.positive_start_index;
            condition(5) = obj.pulse1_state.positive_start_index - obj.pulse2_state.positive_start_index;
            condition(6) = obj.pulse1_state.max_positive_v;
            condition(7) = obj.pulse1_state.max_negative_v;
            condition(8) = obj.pulse2_state.max_positive_v;
            condition(9) = obj.pulse2_state.max_negative_v;
            condition(10) = obj.pulse1_state.transition_p+70;
            condition(11) = obj.pulse2_state.transition_p+70;

            if(obj.debug)
                fprintf('Trigger \n %d, %d, %d, %d, %d,  \n ... %.2f, %.2f, %.2f, %.2f  ... %.1f %.1f\n', condition);
            end
            condition_sum = 0;
            condition_sum = condition_sum + (condition(1) == 3);% patch state
            condition_sum = condition_sum + (condition(2) <= 1);% zero crossing difference
            condition_sum = condition_sum + (condition(3) >= 6);% pulse 2 negative start
            condition_sum = condition_sum + (condition(3) <= 13);% pulse 2 negative start
            condition_sum = condition_sum + (condition(4) >= 11);% pulse 2 positive start
            condition_sum = condition_sum + (condition(4) <= 28);% pulse 2 positive start
            condition_sum = condition_sum + (condition(5) > 3);% difference of positive start
            condition_sum = condition_sum + (condition(11) > 27); % state transition power

            detected = condition_sum == 8;% number of conditions
        end
    end
end

