import unittest

from matplotlib.figure import Figure

from current_loop_tuner.plot_helpers import add_combined_legend


class PlotHelpersTest(unittest.TestCase):
    def test_combined_legend_includes_right_axis_speed_line(self):
        figure = Figure()
        voltage_axis = figure.add_subplot(111)
        speed_axis = voltage_axis.twinx()
        voltage_axis.plot([0, 1], [0, 1], label="Vd")
        voltage_axis.plot([0, 1], [1, 0], label="Vq")
        speed_axis.plot([0, 1], [0, 2], label="Speed")

        legend = add_combined_legend(voltage_axis, (speed_axis,))

        self.assertEqual([text.get_text() for text in legend.get_texts()], ["Vd", "Vq", "Speed"])


if __name__ == "__main__":
    unittest.main()
