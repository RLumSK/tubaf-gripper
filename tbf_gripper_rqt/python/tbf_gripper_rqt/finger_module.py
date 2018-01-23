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

# http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin
import os
import argparse
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

from robotiq_s_model_control.msg import SModel_robot_input as inputMsg
from robotiq_s_model_control.msg import SModel_robot_output as outputMsg


"""@package finger_module
This package gives a controller (RobotiqFinger) and model (RobotiqFingerModel) for the GUI described in ../../resource/RobotiqFinger.ui
The state provided by the Robotiq ROS package via /SModelRobotInput (robotiq_s_model_control.msg.SModel_robot_input) is
passed to the GUI and the changes made their published via /SModelRobotOutput (robotiq_s_model_control.msg.SModel_robot_output).
The design follows the idea of the MVC pattern.
@author: Steve Grehl
"""

class RobotiqFinger(Plugin):
    """
    A instance of this class works as controller for a RobotiqFinger, managing the communication between ROS and Qt using
    a RobotiqFingerModel.
    """
    def __init__(self, context):
        """
        Default constructor loading widget and model as well as setting up the communication using signals and slots
        @param context: Qt context of the plugin (only used by base class)
        @return: initialized instance of RobotiqFinger
        """
        super(RobotiqFinger, self).__init__(context)
        strFingerID = 'A' #self._parse_args(context.argv())
        # Give QObjects reasonable names
        self.setObjectName('RobotiqFinger_'+strFingerID)

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
        ui_file = os.path.join(rospkg.RosPack().get_path('tbf_gripper_rqt'), 'resource', 'RobotiqFinger.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget, {'RobotiqFinger': RobotiqFinger})
        # Give QObjects reasonable names
        self._widget.setObjectName('RobotiqFinger_'+strFingerID)
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # initilize model
        self.model = RobotiqFingerModel(strFingerID)

        # connect signal and slots
        self._widget.sb_rFR.valueChanged.connect(self.model.onForceRequestChanged)
        self._widget.sb_rPR.valueChanged.connect(self.model.onPositionRequestChanged)
        self._widget.sb_rSP.valueChanged.connect(self.model.onSpeedRequestChanged)

        self.model.atNewPR.connect(self._widget.lcd_gPR.display)
        self.model.atNewCU.connect(self._widget.lcd_gCU.display)
        self.model.atNewDT.connect(self._widget.lbl_gDT_value.setText)
        self.model.atNewPO.connect(self._widget.lcd_gPO.display)

    def shutdown_plugin(self):
        """
        unregister all publishers (not implemented)
        @return: -
        """
        self.model.shutdown()
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
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """
        NOT IMPLEMENTED YET
        @param plugin_settings: -
        @param instance_settings: -
        @return: -
        """
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    # from: https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_bag/src/rqt_bag/bag.py
    def _parse_args(self, argv):
        """
        Parsing arguments given via the argparse package
        @param argv: arguments that should be parsed
        @return:
        """
        parser = argparse.ArgumentParser(prog='tbf_hand_rqt', add_help=False)
        RobotiqFinger.add_arguments(parser)
        return parser.parse_args(argv)

    @staticmethod
    def _isvalidID(parser, arg):
        """
        checking if the given argument is a id of a finger (A,B,C,S)
        @param parser: parser that checks the arguments
        @param arg: argument
        @return: argument or exits the program
        """
        if arg == 'A' or arg == 'B' or arg == 'C' or arg == 'S':
            return arg
        else:
            parser.error("Invalid finger_id"+arg)

    @staticmethod
    def add_arguments(parser):
        """
        Parsing arguments to find the finger_id
        @param parser: parser provided by the argparse package?
        @return: -
        """
        group = parser.add_argument_group('Options for RobotiqFinger plugin')
        group.add_argument('finger_id', type=lambda x: RobotiqFinger._isvalidID(parser, x),
                           nargs='*', default=['A'], help='A, B, C, S - ID of the finger')

class RobotiqFingerModel(QtCore.QObject):
    """
    A instance of this class holds the data model for one finger of the Robotiq gripper.
    It holds the necessary signals and slots to communicate with the RobotiqFinger-GUI. The RobotiqFinger-controller establishes the connection.
    """

    # Qt signals
    atNewPR = QtCore.Signal(int)  # new requested position
    atNewDT = QtCore.Signal(str)  # new collision state detected
    atNewPO = QtCore.Signal(int)  # new actual position
    atNewCU = QtCore.Signal(int)  # new current consumption (approx. I = 0.1 * gCU [mA])

    def __init__(self, strID, parentModel=None):
        """
        Default constructor - initializes the data model of a finger, setting all register to 0 and subscribing to 'SModelRobotInput'
        as well as publishing messages via 'SModelRobotOutput'
        @param strID: ID of the finger (A - middle, B, C, S)
        @param parentModel: instance of RobotiqHandModel or None
        @return: instance of this class
        """
        super(RobotiqFingerModel, self).__init__()
        self.strID = strID

        self.gPR = 0
        self.gDT = 0
        self.gPO = 0
        self.gCU = 0
        self.rFR = 0
        self.rPR = 0
        self.rSP = 0

        self.dctActionToSend = {
            "A": self._forA,
            "B": self._forB,
            "C": self._forC,
            "S": self._forS}

        self.dctActionAtReceive = {
            "A": self._reveivedA,
            "B": self._reveivedB,
            "C": self._reveivedC,
            "S": self._reveivedS}

        self.parentModel = parentModel
        # ROS
        # dynamic topic names
        input_topic = rospy.get_param('~robotiq_hand_sub_topic', default='SModelRobotInput')
        joint_topic = rospy.get_param('~robotiq_hand_pub_topic', default='SModelRobotOutput')
        self.subscriber = rospy.Subscriber(input_topic, inputMsg, self.onReceivedROSMessage)
        if self.parentModel is None:
            self.publisher = rospy.Publisher(joint_topic, outputMsg, queue_size=10)
            rospy.loginfo("RobotiqFingerModel.__init__: publishing at: "+joint_topic)
        else:
            self.publisher = None
            rospy.loginfo("RobotiqFingerModel.__init__: publishing at: -")


    @QtCore.Slot(int)
    def onForceRequestChanged(self, newValue):
        """
        from the GUI a new rFR (final grasping force) was received and will be published via ROS
        Minimum force: 15 N
        Maximum force: 60 N
        Force / count: 0.175 N (approximate value, relation non-linear)
        @param newValue: requested force (0-255)
        @return: None
        """
        self.rFR = newValue
        self.sendMessageToHandViaROS()

    @QtCore.Slot(int)
    def onSpeedRequestChanged(self, newValue):
        """
        from the GUI a new rSP (closing/opening speed) was received and will be published via ROS
        Minimum speed: 22 mm/s
        Maximum speed: 110 mm/s
        Speed / count: 0.34 mm/s
        @param newValue: finger speed (0-255)
        @return: None
        """
        self.rSP = newValue
        self.sendMessageToHandViaROS()

    @QtCore.Slot(int)
    def onPositionRequestChanged(self, newValue):
        """
        from the GUI a new rPR (requested position) was received and will be published via ROS
        In order to protect the Gripper from geometric interferences, several software limits are implemented and
        therefore some positions are not reachable. When a finger reaches the software limit, the Gripper status will
        indicate that the requested position has been reached. This is because the requested position is internally
        replaced by the software limit.
        See http://support.robotiq.com/pages/viewpage.action?pageId=590044 figure 4.4.1
        @param newValue: requested position (0-255)
        @return: None
        """
        self.rPR = newValue
        self.sendMessageToHandViaROS()

    def onReceivedROSMessage(self, msg):
        """
        callback for the ROS subscriber
        after receiving a ROS message the registers for the corresponding finger, identified by self.strID, are set and
        the GUI is updated afterwards
        @param msg: ROS message of the type 'SModelRobotInput'
        @return: None
        """
        rospy.logdebug("tbf_gripper_rqt.finger_module.py@RobotiqFingerModel.onReceivedROSMessage(): received message="+str(msg))
        self.dctActionAtReceive.get(self.strID)(msg)
        self.afterReceivedROSMessage()

    def afterReceivedROSMessage(self):
        """
        update the GUI, or other listening classes, via Qt-signals
        @return: None
        """
        rospy.logdebug("tbf_gripper_rqt.finger_module.py@RobotiqFingerModel.afterReceivedROSMessage(): "
                      "sending signals to Qt")
        self.atNewPR[int].emit(self.gPR)
        self.atNewDT[str].emit(self.gDT)
        self.atNewPO[int].emit(self.gPO)
        self.atNewCU[int].emit(self.gCU)

    def sendMessageToHandViaROS(self):
        """
        after the user changed a register, this should be published via ROS either using the own publisher, or if
        available a publisher of a parent model
        @return: None
        """
        if self.publisher is not None:
            msg = outputMsg()
            msg = self.dctActionToSend.get(self.strID)(msg)
            self.publisher.publish(msg)
        elif self.parentModel is not None:
            self.parentModel.sendROSMessage()

    def shutdown(self):
        """
        called when the model will be destroyed - unregister the publisher, if available
        @return: None
        """
        if self.publisher is not None:
            self.publisher.unregister()
        if self.subscriber is not None:
            self.subscriber.unregister()

    def _forA(self, msg):
        #rospy.logwarn("hand_module.py@RobotiqFingerModel._forA(): type="+str(type(self.rFR)))
        msg.rFRA = self.rFR
        msg.rSPA = self.rSP
        msg.rPRA = self.rPR
        return msg

    def _forB(self, msg):
        msg.rFRB = self.rFR
        msg.rSPB = self.rSP
        msg.rPRB = self.rPR
        return msg

    def _forC(self, msg):
        msg.rFRC = self.rFR
        msg.rSPC = self.rSP
        msg.rPRC = self.rPR
        return msg

    def _forS(self, msg):
        msg.rFRS = self.rFR
        msg.rSPS = self.rSP
        msg.rPRS = self.rPR
        return msg

    def _reveivedA(self, msg):
        self.gPR = msg.gPRA
        self.gDT = RobotiqFingerModel.parseDetectedStatus(msg.gDTA)
        self.gPO = msg.gPOA
        self.gCU = msg.gCUA

    def _reveivedB(self, msg):
        self.gPR = msg.gPRB
        self.gDT = RobotiqFingerModel.parseDetectedStatus(msg.gDTB)
        self.gPO = msg.gPOB
        self.gCU = msg.gCUB

    def _reveivedC(self, msg):
        self.gPR = msg.gPRC
        self.gDT = RobotiqFingerModel.parseDetectedStatus(msg.gDTC)
        self.gPO = msg.gPOC
        self.gCU = msg.gCUC

    def _reveivedS(self, msg):
        self.gPR = msg.gPRS
        self.gDT = RobotiqFingerModel.parseDetectedStatus(msg.gDTS)
        self.gPO = msg.gPOS
        self.gCU = msg.gCUS

    @staticmethod
    def parseDetectedStatus(int_value):
        """
        translate the gDT state to a string
        @param int_value: value at register gDT
        @return: string with the state of a finger
        """
        if(int_value == 0):
            return "in motion"
        elif (int_value == 1):
            return "contact (opening)"
        elif (int_value == 2):
            return "contact (closing)"
        elif (int_value == 3):
            return "as requested"
        return "ERROR"
