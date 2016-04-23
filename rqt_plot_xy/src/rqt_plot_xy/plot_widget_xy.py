#!/usr/bin/env python

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

import os
import rospkg
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QIcon, QMenu, QWidget

import rospy

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

from .rosplot import ROSData, RosPlotException


def is_valid_base_topic(topic_name):
    base_topic_type, real_topic_name, _ = topic_helpers.get_topic_type(topic_name)
    if base_topic_type is None:
        message = "topic %s does not exist" % (topic_name)
        return False, None, message
    relative_topic_path = topic_name[len(real_topic_name) + 1:]

    fields = [f for f in relative_topic_path.split('/') if f]

    slot_type, _, _ = roslib.msgs.parse_type(base_topic_type)
    parent_field_class = roslib.message.get_message_class(slot_type)
    is_array = False
    for i in range(len(fields)):
        field = fields[i]

        # parse the field name for an array index
        try:
            field, _, field_index = roslib.msgs.parse_type(field)
        except roslib.msgs.MsgSpecException:
            message = "invalid field %s in topic %s" % (field, real_topic_name)
            return False, None, message

        if field not in getattr(parent_field_class, '__slots__', []):
            message = "no field %s in topic %s" % (relative_topic_path, real_topic_name)
            return [], None, message
        slot_type = parent_field_class._slot_types[parent_field_class.__slots__.index(field)]
        slot_type, slot_is_array, array_size = roslib.msgs.parse_type(slot_type)
        is_array = slot_is_array and field_index is None
        if is_array and not i == len(fields) - 1:
            message = "array field '%s' found, but is not last in topic name: %s" % (field, topic_name)
            return False, None, message

        parent_field_class = topic_helpers.get_type_class(slot_type)

    if not is_array:
        message = "'%s' is not a valid array topic." % topic_name
        return False, None, message
    message = "'%s' is a valid array base topic of type: '%s'" % (topic_name, parent_field_class)
    return True, parent_field_class, message


def get_plottable_fields(base_topic_name, array_class):
    numeric_fields = []
    for i, slot in enumerate(array_class.__slots__):
        slot_type = array_class._slot_types[i]
        slot_type, is_array, array_size = roslib.msgs.parse_type(slot_type)
        slot_class = topic_helpers.get_type_class(slot_type)
        if slot_class in (int, float) and not is_array:
            numeric_fields.append("%s/%s" % (base_topic_name, slot))
        elif slot_class not in (str, bool):
            plottable_fields, _ = get_plottable_fields("%s/%s" % (base_topic_name, slot), slot_class)
            numeric_fields.extend(plottable_fields)
    message = ""
    if len(numeric_fields) > 0:
        message = "%d plottable fields found" % len(numeric_fields)
    else:
        message = "No plottable fields found"
    numeric_fields.sort()
    return numeric_fields, message


class PlotWidgetXY(QWidget):
    _redraw_interval = 40

    def __init__(self, initial_topics=None, start_paused=False):
        super(PlotWidgetXY, self).__init__()
        self.setObjectName('PlotWidget')

        self._initial_topics = initial_topics

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_plot_xy'), 'resource', 'plot.ui')
        loadUi(ui_file, self)
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('list-add'))
        self.remove_topic_button.setIcon(QIcon.fromTheme('list-remove'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = None

        self.subscribe_topic_button.setEnabled(False)
        if start_paused:
            self.pause_button.setChecked(True)

        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics()
        self.topic_edit.setCompleter(self._topic_completer)

        self._rosdata = {}
        self._remove_topic_menu = QMenu()

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)

    def switch_data_plot_widget(self, data_plot):
        self.enable_timer(enabled=False)

        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()

        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(self.autoscale_checkbox.isChecked())

        # setup drag 'n drop
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent

        if self._initial_topics:
            for topic_name in self._initial_topics:
                self.add_topic(*topic_name.split(" - "))
            self._initial_topics = None
        else:
            for topic_name, rosdata in self._rosdata.items():
                data_x, data_y = rosdata.next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        self._subscribed_topics_changed()

    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        # get topic name
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or len(event.source().selectedItems()) == 0:
                qWarning(
                        'Plot.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            topic_name = item.data(0, Qt.UserRole)
            if topic_name == None:
                qWarning('Plot.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
                return
        else:
            topic_name = str(event.mimeData().text())

        # check for plottable field type
        # TODO update this call
        plottable, message = is_plottable(topic_name)
        if plottable:
            event.acceptProposedAction()
        else:
            qWarning('Plot.dragEnterEvent(): rejecting: "%s"' % (message))

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.add_topic(topic_name)

    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completer.update_topics()

        # Correct topic name
        if len(topic_name) > 0 and topic_name[-1] == "/":
            topic_name = topic_name[:-1]
        if len(topic_name) > 2 and topic_name[-2:] == "[]":
            topic_name = topic_name[:-2]

        self.x_field.clear()
        self.y_field.clear()

        # Test base_topic
        valid_base_topic, array_class, message = is_valid_base_topic(topic_name)
        if valid_base_topic:
            fields, message = get_plottable_fields("", array_class)
            if len(fields) > 0:
                self.x_field.addItem("None")
                self.y_field.addItem("None")
                for field in fields:
                    self.x_field.addItem(field)
                    self.y_field.addItem(field)

        self.subscribe_topic_button.setToolTip(message)

    @Slot(str)
    def on_x_field_currentIndexChanged(self, text):
        self.subscribe_topic_button.setEnabled(self.x_field.currentIndex() or self.y_field.currentIndex())

    @Slot(str)
    def on_y_field_currentIndexChanged(self, text):
        self.subscribe_topic_button.setEnabled(self.x_field.currentIndex() or self.y_field.currentIndex())

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topic_edit.text()), str(self.x_field.currentText()), str(self.y_field.currentText()))

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot(bool)
    def on_autoscale_checkbox_clicked(self, checked):
        self.data_plot.autoscroll(checked)
        if checked:
            self.data_plot.redraw()

    @Slot()
    def on_clear_button_clicked(self):
        self.clear_plot()

    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        self.data_plot.update_values(topic_name, data_x, data_y)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot.redraw()

    def _subscribed_topics_changed(self):
        self._update_remove_topic_menu()
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._rosdata)
        self.data_plot.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for keyname in sorted(self._rosdata.keys()):
            action = QAction(keyname, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(keyname))
            self._remove_topic_menu.addAction(action)

        if len(self._rosdata) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)

    def add_topic(self, topic_name, x_field, y_field):
        topics_changed = False
        keyname = topic_name + " - " + x_field + " - " + y_field
        print(keyname)
        if keyname in self._rosdata:
            qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
            return
        self._rosdata[keyname] = ROSData(topic_name, x_field, y_field)
        if self._rosdata[keyname].error is not None:
            qWarning(str(self._rosdata[keyname].error))
            del self._rosdata[keyname]
        else:
            data_x, data_y = self._rosdata[keyname].next()
            self.data_plot.add_curve(keyname, keyname, data_x, data_y)
            topics_changed = True

        if topics_changed:
            self._subscribed_topics_changed()

    def remove_topic(self, keyname):
        self._rosdata[keyname].close()
        del self._rosdata[keyname]
        self.data_plot.remove_curve(keyname)

        self._subscribed_topics_changed()

    def clear_plot(self):
        for keyname, _ in self._rosdata.items():
            self.data_plot.clear_values(keyname)
        self.data_plot.redraw()

    def clean_up_subscribers(self):
        for keyname, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(keyname)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
