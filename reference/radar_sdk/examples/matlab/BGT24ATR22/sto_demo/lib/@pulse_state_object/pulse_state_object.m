classdef pulse_state_object
    %PULSE_STATE_OBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        positive_start_fcount
        negative_start_fcount
        max_positive_v
        max_negative_v
        max_pos_velocity_fcount
        max_neg_velocity_fcount
        last_velocity
        acceleration
        state
        max_power
        max_power_fcount
        aggregate_power
    end
    
    methods
        function obj = pulse_state_object()
            %PULSE_STATE_OBJECT Construct an instance of this class
            %   Detailed explanation goes here
            obj.positive_start_fcount = 0;
            obj.negative_start_fcount = 0;
            obj.last_velocity = 0;
            obj.acceleration = 0;
            obj.state = 0;

            obj.max_positive_v = 0;
            obj.max_negative_v = 0;
            obj.max_pos_velocity_fcount = 0;
            obj.max_neg_velocity_fcount = 0;
            obj.max_power = 0;
            obj.aggregate_power = 0;
            obj.max_power_fcount = 0;
        end
        
        function [trig, obj] = update(obj, velocity, power)
            %UPDATE Summary of this method goes here
            %   Detailed explanation goes here
            trig = 0;
            obj.acceleration = velocity-obj.last_velocity;
            obj.last_velocity = velocity;
            obj.negative_start_fcount = obj.negative_start_fcount + 1;
            obj.positive_start_fcount = obj.positive_start_fcount + 1;
            obj.max_pos_velocity_fcount = obj.max_pos_velocity_fcount + 1;
            obj.max_neg_velocity_fcount = obj.max_neg_velocity_fcount + 1;
            obj.max_power_fcount = obj.max_power_fcount +1;
            switch (obj.state)
                case 0
                    if(velocity > 0)
                        obj.state = 1;
                        obj.max_positive_v = 0;
                        obj.positive_start_fcount = 0;
                        obj.max_power = 0;
                        obj.aggregate_power = 0;
                    end
                case 1
                    if(velocity < 0)
                        obj.state = 3; % clear pos-neg transition
                        obj.max_negative_v = 0;
                        obj.negative_start_fcount = 0;
                    elseif(velocity == 0)
                        obj.state = 2; % unclear pos-neg transition                        
                    end
                    if(velocity > obj.max_positive_v)
                        obj.max_positive_v = velocity;
                        obj.max_pos_velocity_fcount = 0;
                    end
                case 2
                    if(velocity < 0)
                        obj.state = 3; % clear pos-neg transition
                        obj.max_negative_v = 0;
                        obj.negative_start_fcount = 0;
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
                        obj.max_neg_velocity_fcount = 0;
                    end
            end
            if(power>obj.max_power)
                obj.max_power = power;
                obj.max_power_fcount = 0;
            end
            obj.aggregate_power = obj.aggregate_power + power;
        end
    end
end

