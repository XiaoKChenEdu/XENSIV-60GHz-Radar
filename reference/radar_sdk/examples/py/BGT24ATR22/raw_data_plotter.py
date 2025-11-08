from matplotlib import pyplot as plt
import numpy as np


def get_number_of_mimose_pulse_configs(frame_config):
    pulse_configs = 0
    for i in range(4):
        if(frame_config.selected_pulse_configs[i]):
            pulse_configs = pulse_configs+1

    return pulse_configs


# -------------------------------------------------
# Presentation
# -------------------------------------------------
class MimosePlotter:
    """
    Draws plots for data - each pulse configuration is in separated plot
    """

    def __init__(self, config):
        """
        class for plotting mimose data
        
        """
        plt.ion()
        self.num_pulse_configs = get_number_of_mimose_pulse_configs(config.FrameConfig[0])
        num_of_samples = config.FrameConfig[0].num_of_samples;
        self.fig, self.ax = plt.subplots(nrows=self.num_pulse_configs, ncols=1)
        self.x = np.arange(num_of_samples)
        self.fig.canvas.mpl_connect('close_event', self.close)

        if(self.num_pulse_configs > 1):
            for i in range(self.num_pulse_configs):
                self.ax[i].set_ylim([0, 1])        
                self.ax[i].set_xlim([0, num_of_samples-1])
                self.ax[i].set_ylabel('Pulse '+str(i))
        else:
            self.ax.set_ylim([0, 1])        
            self.ax.set_xlim([0, num_of_samples-1])
            self.ax.set_ylabel('Pulse 0')
        self.first_draw = True
        self._is_window_open = True
        self.fig.suptitle('Close plot to exit application')

    def _draw_first_time(self, frame):
        """ Create common plots for IF data and as well scale it in same way

        :return:
        """
        if(self.num_pulse_configs > 1):
            for i in range(self.num_pulse_configs):
                self.ax[i].plot(self.x, np.real(frame[0][i]), 'b')
                self.ax[i].plot(self.x, np.imag(frame[0][i]), 'r')
        else:
            self.ax.plot(self.x, np.real(frame[0][0]), 'b')
            self.ax.plot(self.x, np.imag(frame[0][0]), 'r')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def _draw_next_time(self, frame):
        """ update plot data

        :param time_domain_data: if data
        :param frequency_domain_data: ft data
        :return:
        """
        # draw time domain plot
        if(self.num_pulse_configs > 1):
            for i in range(self.num_pulse_configs):
                self.ax[i].lines[0].set_ydata(np.real(frame[0][i]))
                self.ax[i].lines[1].set_ydata(np.imag(frame[0][i]))
        else:
            self.ax.lines[0].set_ydata(np.real(frame[0][0]))
            self.ax.lines[1].set_ydata(np.imag(frame[0][0]))
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def draw(self, frame):
        """draw initial elements and update
        resets background to white
        :param facecolors:
        :param raw_data:
        :param ft_data:
        :param info_text:
        :return:
        """
        if self.first_draw:
            self._draw_first_time(frame)
            self.first_draw = False
        else:
            self._draw_next_time(frame)

    def close(self, event = None):
        if(self._is_window_open):
            self._is_window_open = False
            plt.close(self.fig)
            plt.close('all') # Needed for Matplotlib ver: 3.4.0 and 3.4.1
            print('Example application closed!')


    def is_open(self):
        return self._is_window_open
