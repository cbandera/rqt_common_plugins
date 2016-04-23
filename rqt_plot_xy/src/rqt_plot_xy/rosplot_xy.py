#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy

from rqt_plot.rosplot import ROSData, RosPlotException, get_topic_type, _field_eval


class ROSDataXY(ROSData):
    """
    Subscriber to ROS topic that buffers incoming data
    """

    def __init__(self, base_topic, x_field, y_field):
        super(ROSDataXY, self).__init__(base_topic, rospy.Time.now())

        topic_type, real_topic, fields = get_topic_type(base_topic)
        if topic_type is not None:
            self.x_field_evals = generate_field_evals(base_topic[len(real_topic) + 1:], x_field)
            self.y_field_evals = generate_field_evals(base_topic[len(real_topic) + 1:], y_field)

    def _ros_cb(self, msg):
        """
        ROS subscriber callback
        :param msg: ROS message data
        """
        try:
            self.lock.acquire()
            try:
                self.buff_x = self._get_data(self.x_field_evals, msg)
                self.buff_y = self._get_data(self.y_field_evals, msg)
            except AttributeError, e:
                self.error = RosPlotException("Invalid topic spec [%s]: %s" % (self.name, str(e)))
        finally:
            self.lock.release()

    def _get_data(self, evals, msg):
        val = msg
        try:
            if not evals:
                return float(val)
            for f in evals:
                val = f(val)
            return [float(v) for v in val]
        except IndexError:
            self.error = RosPlotException("[%s] index error for: %s" % (self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException("[%s] value was not numeric: %s" % (self.name, val))


def _array_eval(field_name):
    """
    :param field_name: name of field to index into, ``str``
    :returns: fn(list of elements)->[fn(field_name for every el in list]
    """

    def fn(f):
        return [getattr(el, field_name) for el in f]

    return fn


def _dummy_eval():
    """
    :returns: fn(msg_field)->[0,1,...,len(msg.field)]
    """

    def fn(f):
        return range(len(f))

    return fn


def generate_field_evals(base_topic, rel_topic):
    try:
        evals = []
        fields = [f for f in base_topic.split('/') if f]
        for f in fields:
            evals.append(_field_eval(f))
        if rel_topic == "None":
            evals.append(_dummy_eval())
        else:
            fields = [f for f in rel_topic.split('/') if f]
            for f in fields:
                evals.append(_array_eval(f))
        return evals
    except Exception, e:
        raise RosPlotException("cannot parse field reference [%s]: %s" % (rel_topic, str(e)))
