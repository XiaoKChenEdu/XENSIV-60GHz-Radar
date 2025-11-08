classdef demo_gui
    % Properties that correspond to app components
    properties
        hfig
        ax1
        ax2
        ax3
        ax4
        still_color
        approach_color
        depart_color
        bg
        bg2
        bg3
        plt
        fft_size
        num_PCs
        half_fft
        half_fft_times_numPCs
        manual_threshold
        auto_threshold
        graph_type
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function obj = demo_gui(varargin)
            p = inputParser;
            addParameter(p, 'config', MimoseConfig());
            addParameter(p, 'fft_size', 64);
            addParameter(p, 'manual_threshold', 5);
            addParameter(p, 'dyn_thresh_factor', 5);
            addParameter(p, 'graph_type', 'manual'); % either 'manual' or 'dynamic' or 'auto'
            addParameter(p, 'auto_threshold', 64);

            parse(p,varargin{:});
            params = p.Results;
            configuration = params.config;
            obj.auto_threshold = params.auto_threshold;
            obj.manual_threshold = params.manual_threshold;
            dyn_thresh_factor = params.dyn_thresh_factor;

            obj.hfig = figure;
            set(obj.hfig, 'MenuBar', 'none', 'NumberTitle', 'off', 'Name', 'BGT24ATR22 dyn presence Demo');

            tiledlayout(obj.hfig, 3, 2);
            obj.ax1 = nexttile;
            obj.ax2 = nexttile;
            obj.ax4 = nexttile;
            obj.ax3 = nexttile([1 2]);

            obj.fft_size = params.fft_size;
            obj.num_PCs = configuration.FrameConfig{1}.selected_pulse_config_0 + ...
                configuration.FrameConfig{1}.selected_pulse_config_1 + ...
                configuration.FrameConfig{1}.selected_pulse_config_2 + ...
                configuration.FrameConfig{1}.selected_pulse_config_3;
            obj.still_color = [0.9 0.9 0.8];
            obj.approach_color = [0 1 0];
            obj.depart_color = [ 0 0 1];
            obj.bg = bar(obj.ax1,1,'Barwidth',30,'FaceColor', obj.still_color);
            obj.bg2 = bar(obj.ax2,1,'Barwidth',30,'FaceColor', obj.still_color);
            obj.bg3 = bar(obj.ax4,1,'Barwidth',30,'FaceColor', obj.still_color);
            set(obj.ax1, 'xtickLabel', [], 'ytickLabel', []);
            set(obj.ax2, 'xtickLabel', [], 'ytickLabel', []);
            set(obj.ax4, 'xtickLabel', [], 'ytickLabel', []);
            obj.plt = plot(obj.ax3, ones(obj.fft_size, obj.num_PCs+1));
            title(obj.ax1,['dynamic factor =' num2str(dyn_thresh_factor)]);
            title(obj.ax2,['auto threshold =' num2str(obj.auto_threshold)]);
            title(obj.ax4,['manual threshold =' num2str(obj.manual_threshold)]);
            obj.half_fft = obj.fft_size/2;
            obj.half_fft_times_numPCs = obj.half_fft*obj.num_PCs;
            if(strcmp(params.graph_type,'dynamic'))
                obj.graph_type = 1;
                obj.plt(end).YData = obj.plt(end).YData*0;
                title(obj.ax3,'signal FFT vs dynamic threshold');
                axis(obj.ax3,[1 obj.fft_size min(-10,-dyn_thresh_factor*2) 10]);
            elseif (strcmp(params.graph_type,'manual'))
                obj.graph_type = 0;
                obj.plt(end).YData = obj.plt(end).YData*obj.manual_threshold;
                title(obj.ax3,'signal FFT vs manual threshold');
                axis(obj.ax3,[1 obj.fft_size 0 obj.manual_threshold*4]);
            else
                obj.graph_type = 0;
                obj.plt(end).YData = obj.plt(end).YData*obj.auto_threshold;
                title(obj.ax3,'signal FFT vs auto trigger threshold');
                axis(obj.ax3,[1 obj.fft_size 0 obj.auto_threshold*2]);
            end
        end

        % Code that executes before app deletion
        function obj = update_plot(obj, doppler_data, doppler_data_valid, dyn_threshold_matrix)
            abs_doppler_data = abs(doppler_data);
            if obj.graph_type == 1
                plot_data = abs_doppler_data-dyn_threshold_matrix;
            else
                plot_data = abs_doppler_data;
            end
            for i = 1:obj.num_PCs
                obj.plt(i).YData = plot_data(i,:);
            end
            abs_doppler_data_valid = abs(doppler_data_valid).';
            [max_fft, idx] = max(abs_doppler_data(:));

            if(sum(abs_doppler_data_valid,'all'))
                if(sum(abs_doppler_data_valid(1:obj.half_fft,:),'all')>sum(abs_doppler_data_valid(obj.half_fft+1:end,:),'all'))
                    obj.bg.FaceColor = obj.approach_color;
                else
                    obj.bg.FaceColor = obj.depart_color;
                end
            else
                obj.bg.FaceColor = obj.still_color;
            end

            if(max_fft>obj.auto_threshold)
                if(idx<=obj.half_fft_times_numPCs)
                    obj.bg2.FaceColor = obj.approach_color;
                else
                    obj.bg2.FaceColor = obj.depart_color;
                end
            else
                obj.bg2.FaceColor = obj.still_color;
            end

            if(max_fft>obj.manual_threshold)
                if(idx<=obj.half_fft_times_numPCs)
                    obj.bg3.FaceColor = obj.approach_color;
                else
                    obj.bg3.FaceColor = obj.depart_color;
                end
            else
                obj.bg3.FaceColor = obj.still_color;
            end
            drawnow;
        end
    end
end
