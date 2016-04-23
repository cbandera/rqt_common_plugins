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

import roslib
from python_qt_binding.QtCore import qWarning, Slot
from python_qt_binding.QtGui import QComboBox
from rqt_plot.plot_widget import PlotWidget
from rqt_py_common import topic_helpers

from .rosplot_xy import ROSDataXY


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

    if len(numeric_fields) > 0:
        message = "%d plottable fields found" % len(numeric_fields)
    else:
        message = "No plottable fields found"

    numeric_fields.sort()
    return numeric_fields, message


class PlotWidgetXY(PlotWidget):
    topic_separator = " - "

    def __init__(self, initial_topics=None, start_paused=False):
        super(PlotWidgetXY, self).__init__(initial_topics, start_paused)

        self.label.setText("ArrayTopic:")
        self.x_field = QComboBox()
        self.x_field.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.x_field.currentIndexChanged.connect(self.on_x_field_currentIndexChanged)
        self.dataPlotControls.insertWidget(2, self.x_field)

        self.y_field = QComboBox()
        self.y_field.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.y_field.currentIndexChanged.connect(self.on_y_field_currentIndexChanged)
        self.dataPlotControls.insertWidget(3, self.y_field)

        self.autoscroll_checkbox.setText("autoscale")

        self.topic_edit.setText("")
        self.topic_edit.setPlaceholderText("/Path/To/VariableArrayField/of/Topic[]/")

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
                self.x_field.setCurrentIndex(0)
                self.y_field.setCurrentIndex(0)
            else:
                valid_base_topic = False

        if valid_base_topic:
            self.topic_edit.setStyleSheet("color: rgb(0, 100, 0);")
        else:
            self.topic_edit.setStyleSheet("color: rgb(0, 0, 0);")

        self.subscribe_topic_button.setToolTip(message)

    @Slot()
    def on_topic_edit_returnPressed(self):
        pass

    @Slot(str)
    def on_x_field_currentIndexChanged(self, text):
        self.subscribe_topic_button.setEnabled(self.x_field.currentIndex() > 0 or self.y_field.currentIndex() > 0)

    @Slot(str)
    def on_y_field_currentIndexChanged(self, text):
        self.subscribe_topic_button.setEnabled(self.x_field.currentIndex() > 0 or self.y_field.currentIndex() > 0)

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        topic_name = self.topic_separator.join([str(self.topic_edit.text()),
                                                str(self.x_field.currentText()),
                                                str(self.y_field.currentText())])
        self.add_topic(topic_name)

    def add_topic(self, topic_name):
        base_topic_name, x_field, y_field = topic_name.split(self.topic_separator)
        topics_changed = False
        if topic_name in self._rosdata:
            qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
            return
        self._rosdata[topic_name] = ROSDataXY(base_topic_name, x_field, y_field)
        if self._rosdata[topic_name].error is not None:
            qWarning(str(self._rosdata[topic_name].error))
            del self._rosdata[topic_name]
        else:
            data_x, data_y = self._rosdata[topic_name].next()
            self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)
            topics_changed = True

        if topics_changed:
            self._subscribed_topics_changed()
