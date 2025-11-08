classdef pulse_state_object
    %PULSE_STATE_OBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        positive_start_index
        negative_start_index
        max_positive_v
        max_negative_v
        transition_p
        last_power
        state
    end
    
    methods
        function obj = pulse_state_object()
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.positive_start_index = 200;
            obj.negative_start_index = 200;
            obj.last_power = -60;
            obj.state = 0;

            obj.max_positive_v = 0;
            obj.max_negative_v = 0;
            obj.transition_p = -60;
        end
        
        function [trig, obj] = update(obj, velocity, power)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            trig = 0;
            obj.negative_start_index = obj.negative_start_index + 1;
            obj.positive_start_index = obj.positive_start_index + 1;
            switch (obj.state)
                case 0
                    if(velocity > 0)
                        obj.state = 1;
                        obj.max_positive_v = 0;
                        obj.positive_start_index = 0;
                    end
                case 1
                    if(velocity < 0)
                        obj.state = 3; % clear pos-neg transition
                        obj.max_negative_v = 0;
                        obj.transition_p = obj.last_power;
                        obj.negative_start_index = 0;
                    elseif(velocity == 0)
                        obj.state = 2; % unclear pos-neg transition                        
                    end
                    if(velocity > obj.max_positive_v)
                        obj.max_positive_v = velocity;
                    end
                case 2
                    if(velocity < 0)
                        obj.state = 3; % clear pos-neg transition
                        obj.max_negative_v = 0;
                        obj.transition_p = obj.last_power;
                        obj.negative_start_index = 0;
                    elseif(velocity >= 0)
                        obj.state = 0;
                    end
                case 3
                    if(velocity >= 0)
                        obj.state = 0;
                        trig = 1;
                    end
                    if(velocity<obj.max_negative_v)
                        obj.max_negative_v = velocity;
                    end
            end
            obj.last_power = power;
        end
    end
end

