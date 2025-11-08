classdef target_process
    %DISTANCE_PROCESS class which provides distance information per doppler
    %bin
    %   Detailed explanation goes here

    properties
        target_coordinates
        target_history
        history_count
    end

    methods
        function obj = target_process(varargin)
            %ANGLE_PROCESS Construct an instance of this class
            %   Detailed explanation goes here
            p = inputParser;
            addParameter(p, 'history_length',5);

            parse(p,varargin{:});
            params = p.Results;

            obj.target_history = zeros(1,params.history_length);
            obj.history_count = 0;
            obj.target_coordinates = [];

        end

        function [target_data_x, target_data_y, obj] = run(obj, data_x, data_y)
            %RUN Summary of this method goes here
            %   Detailed explanation goes here
            
            %%  estimation
            if(~isempty(data_x))
                data_points = complex(data_x, data_y);
                [~, mid_data_point] = min(sum(abs(data_points - data_points.')));
                target_data = complex(data_x(mid_data_point), data_y(mid_data_point));
                obj.target_history(mod(obj.history_count,length(obj.target_history))+1) = target_data;
                obj.history_count = obj.history_count+1;
            end
            
            if(obj.history_count>=length(obj.target_history))
                obj.target_coordinates = median(obj.target_history);
            end

            target_data_x = real(obj.target_coordinates);
            target_data_y = imag(obj.target_coordinates);

        end

    end

end
