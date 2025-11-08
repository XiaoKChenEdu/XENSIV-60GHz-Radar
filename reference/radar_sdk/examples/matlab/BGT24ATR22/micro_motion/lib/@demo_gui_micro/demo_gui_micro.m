classdef demo_gui_micro
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
        function app = demo_gui_micro(varargin)
            p = inputParser;
            addParameter(p, 'history_length', 40);
            
            parse(p,varargin{:});
            params = p.Results;

            history_length = params.history_length;

            app.hfig = figure;
            set(app.hfig, 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'BGT24ATR22 Micro Motion Demo');
            tiledlayout(app.hfig, 4, 4);
            app.ax1 = nexttile([3, 4]);
            app.ax2 = nexttile([1, 2]);
            app.ax3 = nexttile([1, 2]);
            
            %%
            app.p_data = plot(app.ax1, nan(1,history_length), nan(1,history_length),'.-','MarkerSize',8);
            axis(app.ax1, [ -1024 1024 -1024 1024]);
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
        function app = update_points(app, data)
            data_x = real(data(:));
            data_y = imag(data(:));
            if (std(data_x)<1500) && (std(data_y)<1500)
                mean_x = round(mean(data_x)/200)*200;
                mean_y = round(mean(data_y)/200)*200;
                axis(app.ax1, [mean_x-1024 mean_x+1024 mean_y-1024 mean_y+1024]);
            end
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

        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.hfig);
        end
    end
end