classdef demo_gui_da
    % Properties that correspond to app components
    properties
        hfig
        ax1
        ax2
        ax3
        p_data
        p_target
        detection_any
        detection_all
        still_color
        approach_color
        depart_color
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = demo_gui_da(varargin)
            p = inputParser;
            addParameter(p, 'max_dist', 3);
            
            parse(p,varargin{:});
            params = p.Results;

            app.hfig = figure;
            set(app.hfig, 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'BGT24ATR22 Distance Angle Demo');
            tiledlayout(app.hfig, 4, 4);
            app.ax1 = nexttile([3, 4]);
            app.ax2 = nexttile([1, 2]);
            app.ax3 = nexttile([1, 2]);
            
            %%
            app.p_data = plot(app.ax1, nan(1,10), nan(1,10),'x','MarkerSize',8);
            hold(app.ax1,"on");
            app.p_target = plot(app.ax1, nan, nan,'o','MarkerSize',8,'MarkerFaceColor','r');
            pgon = polyshape([-0.2 -0.2 0.2 0.2],[0 0.1 0.1 0]);

            if(params.max_dist<4)
                grid_step = 0.5;
            else
                grid_step = 1;
            end

            meter_range = 0:grid_step:params.max_dist+1;
            theta = 0:5:180;
            theta_rad = deg2rad(theta);
            circular_grid_x = zeros(length(meter_range),length(theta_rad));
            circular_grid_y = zeros(length(meter_range),length(theta_rad));
            for i = 1:length(meter_range)
                [circular_grid_x(i,:), circular_grid_y(i,:)] = pol2cart(theta_rad, ones(1,length(theta_rad))*meter_range(i));
            end
            if(grid_step<1)
                plot(app.ax1, circular_grid_x(2:2:end,:)',circular_grid_y(2:2:end,:)','color',[0.9 0.9 0.9]);
                plot(app.ax1, circular_grid_x(1:2:end,:)',circular_grid_y(1:2:end,:)','color',[0.7 0.7 0.7]);
            else
                plot(app.ax1, circular_grid_x',circular_grid_y','color',[0.7 0.7 0.7]);
            end
            straight_grid_x = circular_grid_x([1 end],1:2:end);
            straight_grid_y = circular_grid_y([1 end],1:2:end);
            plot(app.ax1, straight_grid_x, straight_grid_y, 'color', [0.75 0.75 0.75]);
            plot(app.ax1, pgon,'FaceColor','red','FaceAlpha',0.1);
            gridmax = max(meter_range);
            axis(app.ax1, [-gridmax gridmax 0 gridmax]);

            %%

            app.still_color = [0.9 0.9 0.8];
            app.approach_color = [0 1 0];
            app.depart_color = [ 0 0 1];

            app.detection_any = bar(app.ax2,1,'Barwidth',30,'FaceColor', app.still_color);
            app.detection_all = bar(app.ax3,1,'Barwidth',30,'FaceColor', app.still_color);
            title(app.ax2,'Movement');
            title(app.ax3,'Micromotion');

            set(app.ax2, 'xtickLabel', []);
            set(app.ax2, 'xtick', []);
            set(app.ax2, 'ytickLabel', []);
            set(app.ax2, 'ytick', []);

            set(app.ax3, 'xtickLabel', []);
            set(app.ax3, 'xtick', []);
            set(app.ax3, 'ytickLabel', []);
            set(app.ax3, 'ytick', []);

        end

        % Code that can be called to update plot axes
        function app = update_points(app, data_x, data_y)
            assert(length(data_x) == length(data_y));
            app.p_data.YData = data_y;
            app.p_data.XData = data_x;
        end

        function app = update_status(app, indices, micro_detect, micro_level)
            if(~isempty(indices))
                app.detection_any.FaceColor =  app.approach_color;
            else
                app.detection_any.FaceColor =  app.still_color;
            end
            if(micro_detect)
                app.detection_all.FaceColor =  app.approach_color;
            else
                app.detection_all.FaceColor =  app.still_color;
            end
            xlabel(app.ax3,sprintf('Level = %i', round(micro_level)));
        end

        function app = update_target(app, target_x, target_y, micro_detect)
            if(micro_detect)
                app.p_target.YData = target_y;
                app.p_target.XData = target_x;
            else
                app.p_target.YData = NaN;
                app.p_target.XData = NaN;
            end
        end

        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.hfig);
        end
    end
end