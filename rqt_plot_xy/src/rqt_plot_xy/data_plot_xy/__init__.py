#!/usr/bin/env python

# Copyright (c) 2014, Austin Hendrix
# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy
from rqt_plot.data_plot import DataPlot


class DataPlotXY(DataPlot):
    def __init__(self, parent=None):
        """Create a new, empty DataPlotXY

        This will raise a RuntimeError if none of the supported plotting
        backends can be found
        """
        super(DataPlotXY, self).__init__(parent)

        self._autoscale_x = DataPlot.SCALE_ALL
        self._autoscale_y = DataPlot.SCALE_ALL
        self._switch_plot_markers(True)

    def _switch_data_plot_widget(self, plot_index, markers_on=False):
        """Internal method for activating a plotting backend by index"""
        super(DataPlotXY, self)._switch_data_plot_widget(plot_index, markers_on)
        if not self._data_plot_widget:
            x_limits = [0.0, 10.0]
            y_limits = [0.0, 10.0]
            self.set_xlim(x_limits)
            self.set_ylim(y_limits)
            self.redraw()

    # interface out to the managing DATA component: load data, update data,
    # etc
    def autoscroll(self, enabled=True):
        """Enable or disable autoscaling of the plot"""
        super(DataPlotXY, self).autoscroll(enabled)
        super(DataPlotXY, self).set_autoscale(x=enabled, y=enabled)

    def update_values(self, curve_id, values_x, values_y):
        """Append new data to an existing curve
        
        `values_x` and `values_y` will be appended to the existing data for
        `curve_id`

        Note that the plot is not redraw automatically; call `redraw()` to make
        any changes visible to the user.
        """
        curve = self._get_curve(curve_id)
        curve['x'] = numpy.array(values_x)
        curve['y'] = numpy.array(values_y)

    # autoscaling:  adjusting the plot bounds fit the data
    # autoscrollig: move the plot X window to show the most recent data
    #
    # There is not really a autoscrolling in our PlotXY since we do not track a variable over time.
    # Instead we are only going to use autoscaling
    def _merged_autoscale(self):
        x_limit = [numpy.inf, -numpy.inf]
        y_limit = [numpy.inf, -numpy.inf]

        if self._autoscale_x:
            for curve_id in self._curves:
                curve = self._curves[curve_id]
                if len(curve['x']) > 0:
                    x_limit[0] = min(x_limit[0], curve['x'].min())
                    x_limit[1] = max(x_limit[1], curve['x'].max())
        else:
            # don't modify limit, or get it from plot
            x_limit = self.get_xlim()
        if self._autoscale_y:
            for curve_id in self._curves:
                curve = self._curves[curve_id]
                if len(curve['y']) > 0:
                    y_limit[0] = min(y_limit[0], curve['y'].min())
                    y_limit[1] = max(y_limit[1], curve['y'].max())
        else:
            # don't modify limit, or get it from plot
            y_limit = self.get_ylim()

        # set sane limits if our limits are infinite
        if numpy.isinf(x_limit[0]):
            x_limit[0] = 0.0
        if numpy.isinf(x_limit[1]):
            x_limit[1] = 1.0
        if numpy.isinf(y_limit[0]):
            y_limit[0] = 0.0
        if numpy.isinf(y_limit[1]):
            y_limit[1] = 1.0

        # pad the min/max
        if self._autoscale_y:
            ymin = y_limit[0]
            ymax = y_limit[1]
            delta = ymax - ymin if ymax != ymin else 0.1
            ymin -= .05 * delta
            ymax += .05 * delta
            y_limit = [ymin, ymax]
        if self._autoscale_x:
            xmin = x_limit[0]
            xmax = x_limit[1]
            delta = xmax - xmin if xmax != xmin else 0.1
            xmin -= .05 * delta
            xmax += .05 * delta
            x_limit = [xmin, xmax]

        # Set new limits
        self.set_xlim(x_limit)
        self.set_ylim(y_limit)

    def save_settings(self, plugin_settings, instance_settings):
        super(DataPlotXY,self).save_settings(plugin_settings,instance_settings)
        instance_settings.set_value('markers_on', self._markers_on)

    def restore_settings(self, plugin_settings, instance_settings):
        super(DataPlotXY,self).restore_settings(plugin_settings,instance_settings)
        markers_on = instance_settings.value('markers_on', True) in [True, 'true']
        self._switch_plot_markers(markers_on)
