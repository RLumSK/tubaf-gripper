#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, TU Bergakademie Freiberg
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
import os
import rospkg
import socket
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput as outputMsg
from hand_module import RobotiqHandModel


class WideGripper(Plugin):
    def __init__(self, context):
        super(WideGripper, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('WideGripper')

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
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file_main = os.path.join(rospkg.RosPack().get_path('tbf_gripper_viz'), 'resource', 'Gripper.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file_main, self._widget, {'WideGripper': WideGripper})
        # Give QObjects reasonable names
        self._widget.setObjectName('WideGripper')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self._widget.setWindowTitle("Wide Gripper Interface")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # initilize model
        self.model = WideGripperModel()

        # connect signal and slots
        self._widget.btn_open.clicked.connect(self.model.openGripper)
        self._widget.btn_close.clicked.connect(self.model.closeGripper)
        self._widget.hsl_position.sliderMoved.connect(self.model.moveGripperTo)

    def shutdown_plugin(self):
        # self._widget.shutdown_all()
        self.model.shutdown()
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


class WideGripperModel(RobotiqHandModel):
    def __init__(self):
        super(WideGripperModel, self).__init__()
        self.mdl_fingerB.shutdown()
        self.mdl_fingerC.shutdown()
        self.mdl_fingerS.shutdown()
        del self.mdl_fingerB
        del self.mdl_fingerC
        del self.mdl_fingerS

        self.rACT = 1
        self.rGLV = 1
        self.mdl_fingerA.rSP = 150
        self.mdl_fingerA.rFR = 200

        self.onMODChanged(2)

    def closeGripper(self):
        self.mdl_fingerA.rPR = 255
        self.rGTO = 1
        self.sendROSMessage()
        self.rGTO = 0

    def openGripper(self):
        self.mdl_fingerA.rPR = 0
        self.rGTO = 1
        self.sendROSMessage()
        self.rGTO = 0

    def moveGripperTo(self, position):
        self.mdl_fingerA.rPR = position
        self.rGTO = 1
        self.sendROSMessage()
        self.rGTO = 0

    def sendROSMessage(self):
        """
        publish the internal state via ROS using a 'SModelRobotOutput' message
        @return: None
        """
        msg = outputMsg()
        msg.rACT = self.rACT
        msg.rMOD = self.rMOD
        msg.rGTO = self.rGTO
        msg.rICF = self.rICF
        msg.rICS = self.rICS
        msg.rATR = self.rATR
        msg.rGLV = self.rGLV
        msg.rPRA = self.mdl_fingerA.rPR
        msg.rSPA = self.mdl_fingerA.rSP
        msg.rFRA = self.mdl_fingerA.rFR
        rospy.loginfo("hand_module.py@RobotiqHandModel.sendROSMessage(): msg="+str(msg))
        self.publisher.publish(msg)

    def shutdown(self):
        """
        unregister publisher and subscriber and make sure everything is suspended
        :return: -
        :rtype: None
        """
        self.mdl_fingerA.shutdown()
        self.publisher.unregister()
        super.subscriber.unregister()


class PinchGripper(Plugin):
    def __init__(self, context):
        super(PinchGripper, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('PinchGripper')

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
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file_main = os.path.join(rospkg.RosPack().get_path('tbf_gripper_viz'), 'resource', 'Gripper.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file_main, self._widget, {'PinchGripper': PinchGripper})
        # Give QObjects reasonable names
        self._widget.setObjectName('PinchGripper')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self._widget.setWindowTitle("Pinch Gripper Interface")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # initilize model
        self.model = PinchGripperModel()

        # connect signal and slots
        self._widget.btn_open.clicked.connect(self.model.openGripper)
        self._widget.btn_close.clicked.connect(self.model.closeGripper)
        self._widget.hsl_position.sliderMoved.connect(self.model.moveGripperTo)

    def shutdown_plugin(self):
        # self._widget.shutdown_all()
        self.model.shutdown()
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


class PinchGripperModel(WideGripperModel):
    def __init__(self):
        super(PinchGripperModel, self).__init__()
        self.onMODChanged(1)  # pinch mode


class BasicGripper(Plugin):
    def __init__(self, context):
        super(BasicGripper, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('BasicGripper')

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
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file_main = os.path.join(rospkg.RosPack().get_path('tbf_gripper_viz'), 'resource', 'Gripper.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file_main, self._widget, {'BasicGripper': BasicGripper})
        # Give QObjects reasonable names
        self._widget.setObjectName('BasicGripper')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self._widget.setWindowTitle("Basic Gripper Interface")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # initilize model
        self.model = BasicGripperModel()

        # connect signal and slots
        self._widget.btn_open.clicked.connect(self.model.openGripper)
        self._widget.btn_close.clicked.connect(self.model.closeGripper)
        self._widget.hsl_position.sliderMoved.connect(self.model.moveGripperTo)

    def shutdown_plugin(self):
        # self._widget.shutdown_all()
        self.model.shutdown()
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


class BasicGripperModel(WideGripperModel):
    def __init__(self):
        super(BasicGripperModel, self).__init__()
        self.onMODChanged(0)  # basic mode