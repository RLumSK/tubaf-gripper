#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, TU Bergakademie Freiberg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin
import os
import rospy
import rospkg
import argparse

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtWidgets, QtGui
from python_qt_binding.QtWidgets import QWidget

import control_msgs.msg
import actionlib


"""@package grippercommandaction_module
This package gives a controller (GcaHand) and model (GcaHandModel) for the GUI described in ../../resource/RobotiqHand.ui
The used backend is provided by the ROS package "robotiq_s_model_action_server" (https://github.com/TAMS-Group/robotiq_s_model_action_server).
It implements an action server - control_msgs/GripperCommand Action
The design follows the idea of the MVC pattern.
@author: Steve Grehl
"""


class GcaHand(Plugin):
    """
    A instance of this class works as controller for a RobotiqHand, managing the communication between ActionServer and
    Qt using a GcaHandModel.
    """
    def __init__(self, context):
        """
        Default constructor loading widget and model as well as setting up the communication using signals and slots
        @param context: Qt context of the plugin (only used by base class)
        @return: initialized instance of RobotiqHand
        """
        super(GcaHand, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GcaHand')
        self._services = []

        # Vorlage:
        # https: // github.com / ros - visualization / rqt_common_plugins / blob / master / rqt_plot / src / rqt_plot / plot.py

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._wdg_main = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file_gca = os.path.join(rospkg.RosPack().get_path('tbf_gripper_viz'), 'resource', 'gca.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file_gca, self._wdg_main, {'GcaMain': GcaHand})
        rospy.loginfo("GcaHand.__init__(): Loaded UI")
        # Give QObjects reasonable names
        self._wdg_main.setObjectName('GcaMain')

        # Set up the ComboBox for the topics
        self._wdg_main.btn_refresh.clicked.connect(self.actualize_topic_list)
        self._wdg_main.btn_refresh.setIcon(QtGui.QIcon(os.path.join(rospkg.RosPack().get_path('tbf_gripper_viz'),
                                                                    'resource', 'refresh.png')))
        self._wdg_main.btn_refresh.setIconSize(QtCore.QSize(24, 24))

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._wdg_main)

        # initilize model
        rospy.loginfo("GcaHand.__init__(): Starting Model")
        self.model = GcaModel()
        self._wdg_main.dsb_gl_position.setMaximum(self.model.max_position)
        self._wdg_main.dsb_gl_position.setMinimum(self.model.min_position)
        self._wdg_main.dsb_gl_position.setSingleStep((self.model.max_position - self.model.min_position)/10.0)

        self._wdg_main.dsb_gl_effort.setMaximum(self.model.max_effort)
        self._wdg_main.dsb_gl_effort.setMinimum(self.model.min_effort)
        self._wdg_main.dsb_gl_effort.setSingleStep((self.model.max_effort - self.model.min_effort)/10.0)

        # connect signal and slots
        rospy.loginfo("GcaHand.__init__(): Connecting...")
        self._connectQtAndModel(self._wdg_main, self.model)
        self.actualize_topic_list()
        self._wdg_main.dsb_gl_effort.setValue(self.model.min_effort + (self.model.max_effort -
                                                                       self.model.min_effort)/2.0)
        self._wdg_main.dsb_gl_position.setValue(self.model.min_position + (self.model.max_position -
                                                                           self.model.min_position)/2.0)
        rospy.loginfo("GcaHand.__init__(): Connected")

    @QtCore.Slot(int)
    def actualize_topic_list(self):
        """
        Get all published topics and actualize the QComboBox
        :return: -
        :rtype: None
        """
        self._wdg_main.cb_service.clear()
        lst_topics = rospy.get_published_topics()
        rospy.logdebug("GcaHand.actualize_topic_list(): Topic should be " +
                          str(control_msgs.msg.GripperCommandActionResult._type))

        for topic in lst_topics:
            rospy.logdebug("GcaHand.actualize_topic_list(): Topic is "+topic[1])
            if topic[1] == str(control_msgs.msg.GripperCommandActionResult._type):
                rospy.loginfo("GcaHand.actualize_topic_list(): append: "+str(topic[0][:-7])+"   type: "+str(topic[1]))
                self._wdg_main.cb_service.addItem(topic[0][:-7])

    def _parse_args(self, argv):
        """
        parsing the context arguments - see: https://github.com/ros-visualization/rqt_common_plugins/blob/master/rqt_plot/src/rqt_plot/plot.py
        :param argv: context arguments
        :type argv:
        :return: args
        :rtype:
        """
        parser = argparse.ArgumentParser(prog='tbf_gripper_rqt', add_help=False)
        GcaHand.add_arguments(parser)
        args = parser.parse_args(argv)

        # convert topic arguments into topic names
        topic_list = []
        for t in args.topics:
            # c_topics is the list of topics to plot
            c_topics = []
            # compute combined topic list, t == '/foo/bar1,/baz/bar2'
            for sub_t in [x for x in t.split(',') if x]:
                # check for shorthand '/foo/field1:field2:field3'
                if ':' in sub_t:
                    base = sub_t[:sub_t.find(':')]
                    # the first prefix includes a field name, so save then strip it off
                    c_topics.append(base)
                    if not '/' in base:
                        parser.error("%s must contain a topic and field name" % sub_t)
                    base = base[:base.rfind('/')]

                    # compute the rest of the field names
                    fields = sub_t.split(':')[1:]
                    c_topics.extend(["%s/%s" % (base, f) for f in fields if f])
                else:
                    c_topics.append(sub_t)
            # #1053: resolve command-line topic names
            import rosgraph
            c_topics = [rosgraph.names.script_resolve_name('tbf_gripper_rqt', n) for n in c_topics]
            if type(c_topics) == list:
                topic_list.extend(c_topics)
            else:
                topic_list.append(c_topics)
        args.topics = topic_list

        return args

    def shutdown_plugin(self):
        """
        unregister all publishers
        @return: None
        """
        #self._widget.shutdown_all()
        self.model.shutdown()
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        """
        NOT IMPLEMENTED YET
        @param plugin_settings: -
        @param instance_settings: -
        @return: -
        """
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        # NOT implemented: self.model.save_settings(plugin_settings, instance_settings)
        # instance_settings.set_value('topics', pack(self._widget._rosdata.keys()))

    def restore_settings(self, plugin_settings, instance_settings):
        """
        NOT IMPLEMENTED YET
        @param plugin_settings: -
        @param instance_settings: -
        @return: -
        """
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        # autoscroll = instance_settings.value('autoscroll', True) in [True, 'true']
        # self._widget.autoscroll_checkbox.setChecked(autoscroll)
        # self.model.autoscroll(autoscroll)
        # self._update_title()
        #
        # if len(self._widget._rosdata.keys()) == 0 and not self._args.start_empty:
        #     topics = unpack(instance_settings.value('topics', []))
        #     if topics:
        #         for topic in topics:
        #             self._widget.add_topic(topic)
        #
        # self._data_plot.restore_settings(plugin_settings, instance_settings)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def _connectQtAndModel(self, widget, model):
        """
        Connect the Signals and Slots between view and model
        :param widget: View designed by the QT-Designer
        :type widget: QWidget
        :param model: Model implemented in Python
        :type model: GcaModel
        :return: -
        :rtype: None
        """
        # From view to model
        widget.btn_gl_start.clicked.connect(model.onNewGoalStart)
        widget.dsb_gl_position.valueChanged.connect(model.onNewGoalPosition)
        widget.dsb_gl_effort.valueChanged.connect(model.onNewGoalEffort)
        widget.cb_service.currentTextChanged.connect(model.onNewServiceSelected)

        # From model to view
        model.atNewFdbPosition.connect(widget.lcd_fdb_position.display)
        model.atNewFdbEffort.connect(widget.lcd_fdb_effort.display)
        model.atNewFdbStalled.connect(widget.chk_fdb_stalled.setChecked)
        model.atNewFdbReached.connect(widget.chk_fdb_reached.setChecked)

        model.atNewResPosition.connect(widget.lcd_res_position.display)
        model.atNewResEffort.connect(widget.lcd_res_effort.display)
        model.atNewResStalled.connect(widget.chk_res_stalled.setChecked)
        model.atNewResReached.connect(widget.chk_res_reached.setChecked)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_plot plugin')
        group.add_argument('-P', '--pause', action='store_true', dest='start_paused',
                           help='Start in paused state')
        group.add_argument('-e', '--empty', action='store_true', dest='start_empty',
                           help='Start without restoring previous topics')
        group.add_argument('topics', nargs='*', default=[], help='Topics to plot')


class GcaModel(QtCore.QObject):
    """
    A instance of this class holds the data model for the GripperCommandAction client.
    It holds the necessary signals and slots to communicate with the GcaHand-GUI. The GcaHand-controller
    establishes the connection.
    """
    atNewFdbPosition = QtCore.Signal(float)  # Position from the Feedback-msg, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html.
    atNewFdbEffort = QtCore.Signal(float)    # Effort from the Feedback-msg, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html.
    atNewFdbStalled = QtCore.Signal(bool)    # Stalled from the Feedback-msg, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html.
    atNewFdbReached = QtCore.Signal(bool)    # Reached goal from the Feedback-msg, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html.
    atNewResPosition = QtCore.Signal(float)  # Position from the Result-msg, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html.
    atNewResEffort = QtCore.Signal(float)    # Effort from the Result-msg, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html.
    atNewResStalled = QtCore.Signal(bool)    # Stalled from the Result-msg, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html.
    atNewResReached = QtCore.Signal(bool)    # Reached goal from the Result-msg, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html.

    def __init__(self):
        """
        Default constructor - initializes the data model of the gripper, setting all register to 0, 4 finger models and
        subscribing to 'SModelRobotInput' as well as publishing messages via 'SModelRobotOutput'
        @return:
        """
        super(GcaModel, self).__init__()

        self.gl_position = 0
        self.gl_effort = 0

        # init ROS
        # rospy.init_node('hand_module', anonymous=True)
        # The plugin should not call init_node as this is performed by rqt_gui_py. The plugin can use any rospy-specific
        # functionality (like Publishers, Subscribers, Parameters). Just make sure to stop timers and publishers,
        # unsubscribe from Topics etc in the shutdown_plugin method.

        # Goal gripper rad size is in range(0.049500 to 1.222000):
        self.min_position = rospy.get_param("~min_position", 0.0495)
        self.max_position = rospy.get_param("~max_position", 1.222)
        # Goal gripper effort in range (40.000000 to 100.000000 N)
        self.min_effort = rospy.get_param("~min_effort", 40.0)
        self.max_effort = rospy.get_param("~max_effort", 100.0)
        self.goal_handle = None
        self.client = None

    def initClient(self, name):
        """
        Init an Action Client with the given name
        :param name: Namespace of the action server
        :type name: str
        :return: -
        :rtype: None
        """
        if self.client is not None:
            self.client.cancel_all_goals()
        self.client = actionlib.SimpleActionClient(name, control_msgs.msg.GripperCommandAction)
        self.client.wait_for_server()

    @QtCore.Slot()
    def onNewGoalStart(self):
        """
        Invoke a new goal via GripperCommand
        :return: -
        :rtype: None
        """
        goal = control_msgs.msg.GripperCommandGoal()
        if self.gl_position > self.min_position:
            if self.gl_position < self.max_position:
                goal.command.position = self.gl_position
            else:
                goal.command.position = self.max_position
        else:
            goal.command.position = self.min_position

        if self.gl_effort > self.min_effort:
            if self.gl_effort < self.max_effort:
                goal.command.max_effort = self.gl_effort
            else:
                goal.command.max_effort = self.max_effort
        else:
            goal.command.max_effort = self.min_effort

        self.goal_handle = self.client.send_goal(goal, done_cb=self.cbResult, feedback_cb=self.cbFeedback)
        if self.client is None:
            rospy.logwarn("GcaModel.onNewGoalStart(): client is None - No Action Server active?")
        self.client.wait_for_result(rospy.Duration(2))

    def cbFeedback(self, msg):
        """
        Feedback from the action server, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html
        :param msg: Feedback message
        :type msg: control_msgs.msg.GripperCommandActionFeedback
        :return: -
        :rtype: None
        """
        rospy.loginfo("GcaModel.cbFeedback(): "+str(msg))
        self.atNewFdbPosition.emit(msg.position)
        self.atNewFdbEffort.emit(msg.effort)
        self.atNewFdbStalled.emit(msg.stalled)
        self.atNewFdbReached.emit(msg.reached_goal)

    def cbResult(self, status, msg):
        """
        Result from the action server, see: http://docs.ros.org/jade/api/control_msgs/html/action/GripperCommand.html
        :param msg: Result message
        :type msg: control_msgs.msg.GripperCommandActionResult
        :return: -
        :rtype: None
        """
        rospy.loginfo("GcaModel.cbResult(): Status: \n"+str(status))
        rospy.loginfo("GcaModel.cbResult(): Result: \n"+str(msg))
        self.atNewResPosition.emit(msg.position)
        self.atNewResEffort.emit(msg.effort)
        self.atNewResStalled.emit(msg.stalled)
        self.atNewResReached.emit(msg.reached_goal)
        self.goal_handle = None

    @QtCore.Slot(str)
    def onNewServiceSelected(self, service):
        """
        Set a new service and unregister the previous one
        :param service: name of the topic
        :type service: String
        :return: None
        :rtype: -
        """
        rospy.loginfo("GcaModel.onNewServiceSelected: new service is "+service)
        if service == "":
            rospy.logwarn("GcaModel.onNewServiceSelected: <empty>")
            return
        if self.client is not None:
            self.client.cancel_all_goals()
        self.goal_handle = None
        self.client = actionlib.SimpleActionClient(service, control_msgs.msg.GripperCommandAction)
        self.client.wait_for_server()
        rospy.loginfo("GcaModel.onNewServiceSelected: "+service)

    @QtCore.Slot(float)
    def onNewGoalPosition(self, new_pos):
        """
        Set new position for the gripper - does not invoke a goal command
        :param new_pos: new gripper position
        :type new_pos: float
        :return: -
        :rtype: None
        """
        if new_pos >= 0.0:
            self.gl_position = new_pos
        else:
            rospy.logwarn("GcaModel.onNewGoalPosition: Illegal position "+str(new_pos))

    @QtCore.Slot(float)
    def onNewGoalEffort(self, new_effort):
        """
        Set new max effort for the gripper - does not invoke a goal command
        :param new_effort: new gripper max effort
        :type new_effort: float
        :return: -
        :rtype: None
        """
        if new_effort >= 0.0:
            self.gl_effort = new_effort
        else:
            rospy.logwarn("GcaModel.onNewGoalEffort: Illegal max effort "+str(new_effort))

    def shutdown(self):
        """
        called when the model will be destroyed - cancel all goals, if any, and destroy all handles
        @return: None
        """
        self.client.cancel_all_goals()
        self.goal_handle = None
        self.client = None
