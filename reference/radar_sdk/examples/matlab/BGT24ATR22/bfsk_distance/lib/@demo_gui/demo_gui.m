classdef demo_gui
    % Properties that correspond to app components
    properties
        hfig
        ax1
        ax2
        ax3
        ax4
        ax5
        ax6
        distance_ant1
        distance_ant2
        level_ant1
        level_ant2
        velocity_ant1
        velocity_ant2
        color_waiting
        color_detected
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = demo_gui(varargin)
            p = inputParser;
            addParameter(p, 'max_range_1', 1.2);
            addParameter(p, 'max_range_2', 1.2);
            addParameter(p, 'v_max', 5);

            parse(p,varargin{:});
            params = p.Results;

            app.hfig = figure;
            set(app.hfig, 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'BGT24ATR22 BFSK Demo');
            tiledlayout(app.hfig, 4, 4);
            app.ax1 = nexttile([3, 2]);
            app.ax2 = nexttile([3, 2]);
            app.ax3 = nexttile;
            app.ax5 = nexttile;
            app.ax4 = nexttile;
            app.ax6 = nexttile;

            app.distance_ant1 = bar(app.ax1, 0, 'b');
            axis(app.ax1, [0.6 1.4 0 params.max_range_1*100]);
            grid(app.ax1,"minor");
            set(app.ax1, 'xtickLabel', []);
            set(app.ax1, 'xtick', []);

            app.distance_ant2 = bar(app.ax2, 0, 'b');
            axis(app.ax2, [0.6 1.4 0 params.max_range_2*100]);
            set(app.ax2, 'xtickLabel', []);
            grid(app.ax2,"minor");
            set(app.ax2, 'xtick', []);
            
            app.level_ant1 = bar(app.ax3, 0,'g','BarWidth',2);
            axis(app.ax3, [0.6 1.4 0 65]);
            set(app.ax3, 'xtick', []);
            set(app.ax3, 'ytick', []);
            set(app.ax3, 'xtickLabel', []);
            set(app.ax3, 'ytickLabel', []);

            app.level_ant2 = bar(app.ax4, 0,'g','BarWidth',2);
            axis(app.ax4, [0.6 1.4 0 65]);
            set(app.ax4, 'xtick', []);
            set(app.ax4, 'ytick', []);
            set(app.ax4, 'xtickLabel', []);
            set(app.ax4, 'ytickLabel', []);

            app.velocity_ant1 = bar(app.ax5, 0,'g','BarWidth',2);
            axis(app.ax5, [0.6 1.4 -params.v_max params.v_max]);
            set(app.ax5, 'xtick', []);
            set(app.ax5, 'ytick', []);
            set(app.ax5, 'xtickLabel', []);
            set(app.ax5, 'ytickLabel', []);

            app.velocity_ant2 = bar(app.ax6, 0,'g','BarWidth',2);
            axis(app.ax6, [0.6 1.4 -params.v_max params.v_max]);
            set(app.ax6, 'xtick', []);
            set(app.ax6, 'ytick', []);
            set(app.ax6, 'xtickLabel', []);
            set(app.ax6, 'ytickLabel', []);

            title(app.ax1, 'Ant 1 Distance (cm)');
            title(app.ax2, 'Ant 2 Distance (cm)');
            title(app.ax3, 'level');
            title(app.ax4, 'level');
            title(app.ax5, 'velocity');
            title(app.ax6, 'velocity');

            xlabel(app.ax1, '0.0 cm');
            xlabel(app.ax2, '0.0 cm');
            xlabel(app.ax3, '-65 dB FS');
            xlabel(app.ax4, '-65 dB FS');
            xlabel(app.ax5, '0.00 m/s');
            xlabel(app.ax6, '0.00 m/s');

            set(app.ax4, 'xtickLabel', []);
            set(app.ax4, 'ytickLabel', []);
            app.color_waiting = [0.65,0.65,0.65];
            app.color_detected = [0.00,1.00,0.00];
            app.level_ant1.FaceColor = app.color_waiting;
            app.level_ant2.FaceColor = app.color_waiting;
            app.velocity_ant1.FaceColor = [0.79,0.80,0.21];
            app.velocity_ant2.FaceColor = [0.79,0.80,0.21];
        end

        % Code that executes before app deletion
        function app = update_plot(app, data, threshold)
            % data = [ dista_m distb_m power_a power_b]
            if(data(3) > threshold)
                app.level_ant1.FaceColor = app.color_detected;
                app.distance_ant1.YData = data(1)*100;
                app.velocity_ant1.YData = data(5);
                dist_text = sprintf('%.1f cm', data(1)*100);
                xlabel(app.ax1, dist_text);
                vel_text = sprintf('%.2f m/s',data(5));
                xlabel(app.ax5, vel_text);
            else
                app.velocity_ant1.YData = 0;
                xlabel(app.ax5, '0.00 m/s');
                app.level_ant1.FaceColor = app.color_waiting;
            end
            pow_text = sprintf('%.1f dB', data(3));
            app.level_ant1.YData = data(3)+65;
            xlabel(app.ax3, pow_text);

            if(data(4) > threshold)
                app.level_ant2.FaceColor = app.color_detected;
                app.distance_ant2.YData = data(2)*100;
                app.velocity_ant2.YData = data(6);
                dist_text = sprintf('%.1f cm', data(2)*100);
                xlabel(app.ax2, dist_text);
                vel_text = sprintf('%.2f m/s',data(6));
                xlabel(app.ax6, vel_text);
            else
                app.velocity_ant2.YData = 0;
                xlabel(app.ax6, '0.00 m/s');
                app.level_ant2.FaceColor = app.color_waiting;
            end
            pow_text = sprintf('%.1f dB FS', data(4));
            app.level_ant2.YData = data(4)+65;
            xlabel(app.ax4, pow_text);
            drawnow;
        end

        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.hfig);
        end
    end
end