classdef demo_gui
    % Properties that correspond to app components
    properties
        hfig
        ax1
        ax2
        ax3
        ax4
        velocity_data
        power_data
        plot_vel
        plot_pow        
        bargraph
        car_img
        image_num
        sDir
        sound_open
        sound_open_fs
        sound_close
        sound_close_fs
        sound_beep
        sound_beep_fs
        frame_history
        color_waiting
        color_kick
        color_sniffing
        color_false
        sniffing_count
        sniffing_thresh
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = demo_gui
            app.frame_history = 40;
            app.hfig = figure;
            set(app.hfig, 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'BGT24ATR22 STO Demo');
            tiledlayout(app.hfig, 3, 2);
            app.ax1 = nexttile([1 2]);
            app.ax2 = nexttile([1 2]);
            app.ax3 = nexttile;
            app.ax4 = nexttile;

            app.velocity_data = zeros(app.frame_history,4);
            app.plot_vel = plot(app.ax1, app.velocity_data);
            axis(app.ax1, [1 app.frame_history -64 64]);
            set(app.ax1, 'xtickLabel', []);
            app.power_data = nan(app.frame_history,2);
            app.plot_pow = plot(app.ax2, app.power_data);
            axis(app.ax2, [1 app.frame_history 0 120]);
            set(app.ax2, 'xtickLabel', []);
            app.bargraph = bar(app.ax3, 1,'g','BarWidth',2);
            axis(app.ax3, [0.5 1.5 0.1 0.9]);
            set(app.ax3, 'xtickLabel', []);
            set(app.ax3, 'ytickLabel', []);


            title(app.ax1, 'Demo: Close plot to quit program');
            xlabel(app.ax1, 'velocity bin detected');
            xlabel(app.ax2, 'power detected');
            title(app.ax3, 'Waiting');

            app.image_num = 2;
            dirname = 'lib\gui_data';
            ext = '.png';
            app.sDir=  dir(fullfile(dirname ,['*' ext]));
            app.car_img = image(app.ax4,imread([app.sDir(app.image_num).folder,'\',app.sDir(app.image_num).name]));
            set(app.ax4, 'xtickLabel', []);
            set(app.ax4, 'ytickLabel', []);
            [app.sound_open, app.sound_open_fs] = audioread("lib\gui_data\open_sound.wav");
            [app.sound_close, app.sound_close_fs] = audioread("lib\gui_data\close_sound.wav");
            [app.sound_beep, app.sound_beep_fs] = audioread("lib\gui_data\beep_sound.wav");
            app.color_false = [0.00,0.45,0.74];
            app.color_waiting = [0.07,0.62,1.00];
            app.color_kick = [0.00,1.00,0.00];
            app.color_sniffing = [0.65,0.65,0.65];
            app.bargraph.FaceColor = app.color_waiting;

            app.sniffing_thresh = 200; % consecutive low power frames before sniffing transition
            app.sniffing_count = 0;
        end

        % Code that executes before app deletion
        function app = update_plot(app, data, kick_det_obj)
            if(data(3)<kick_det_obj.power_threshold_rear)
                data(1) = 0;
            end
            if(data(4)<kick_det_obj.power_threshold_down)
                data(2) = 0;
            end
            app.velocity_data = [app.velocity_data(2:end,:); ...
                data(1) data(2) kick_det_obj.pulse1_state.acceleration kick_det_obj.pulse2_state.acceleration];
            app.power_data = [app.power_data(2:end,:); 20*log10(data(3)) 20*log10(data(4)) ];
            app.plot_vel(1).YData = app.velocity_data(:,1);
            app.plot_vel(2).YData = app.velocity_data(:,2);
            app.plot_vel(3).YData = app.velocity_data(:,3);
            app.plot_vel(4).YData = app.velocity_data(:,4);
            app.plot_pow(1).YData = app.power_data(:,1);
            app.plot_pow(2).YData = app.power_data(:,2);
            

            if(data(3) < kick_det_obj.power_threshold_rear)
                app.sniffing_count = app.sniffing_count + 1;
                if(app.sniffing_count >= app.sniffing_thresh)
                    app.ax3.Title.String = 'Sniffing';
                    app.bargraph.FaceColor = app.color_sniffing;
                end
            else
                app.sniffing_count = 0;
                app.ax3.Title.String = 'Waiting';
                app.bargraph.FaceColor = app.color_waiting;
            end

            drawnow;
        end

        function app = kick_detected(app)
            % check if trunk is closed
            title(app.ax3, 'Kick');
            app.bargraph.FaceColor = app.color_kick;
            trunk_closed = isequal(app.image_num, 2);
            if(trunk_closed)
                image_routine = [6 3 1 4 5];
                sound_file = app.sound_open;
                sound_fs = app.sound_open_fs;
                sound_trig = 2;
            else
                image_routine = [7 4 1 3 2];
                sound_file = app.sound_close;
                sound_fs = app.sound_close_fs;
                sound_trig = 1;
            end
            sound(app.sound_beep, app.sound_beep_fs);
            for i = 1:length(image_routine)
                app.car_img = image(app.ax4,imread([app.sDir(image_routine(i)).folder,'\',app.sDir(image_routine(i)).name]));
                set(app.ax4, 'xtickLabel', []);
                set(app.ax4, 'ytickLabel', []);
                pause(0.5);
                if(i==sound_trig)
                    sound(sound_file, sound_fs);
                end
            end
            app.image_num = image_routine(end);
            app.bargraph.FaceColor = app.color_waiting;
            title(app.ax3, 'Waiting');
        end

        function mode = get_sto_mode(app)
            mode = strcmp(app.ax3.Title.String, 'Waiting');
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.hfig);
        end
    end
end