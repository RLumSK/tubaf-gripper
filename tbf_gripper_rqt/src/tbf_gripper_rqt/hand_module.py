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
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding import QtCore, QtGui

from robotiq_s_model_control.msg import SModel_robot_input  as inputMsg
from robotiq_s_model_control.msg import SModel_robot_output as outputMsg

from finger_module import RobotiqFinger, RobotiqFingerModel

"""@package finger_module
This package gives a controller (RobotiqHand) and model (RobotiqHandModel) for the GUI described in ../../resource/RobotiqHand.ui
The state provided by the Robotiq ROS package via /SModelRobotInput (robotiq_s_model_control.msg.SModel_robot_input) is
passed to the GUI and the changes made their published via /SModelRobotOutput (robotiq_s_model_control.msg.SModel_robot_output).
The design follows the idea of the MVC pattern.
@author: Steve Grehl
"""

class RobotiqHand(Plugin):
    """
    A instance of this class works as controller for a RobotiqHand, managing the communication between ROS and Qt using
    a RobotiqHandModel.
    """
    def __init__(self, context):
        """
        Default constructor loading widget and model as well as setting up the communication using signals and slots
        @param context: Qt context of the plugin (only used by base class)
        @return: initialized instance of RobotiqHand
        """
        super(RobotiqHand, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RobotiqHand')

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
        self._wdg_fingerA = QWidget()
        self._wdg_fingerB = QWidget()
        self._wdg_fingerC = QWidget()
        self._wdg_fingerS = QWidget()
        self._wdg_main = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file_finger = os.path.join(rospkg.RosPack().get_path('tbf_gripper_rqt'), 'resource', 'RobotiqFinger.ui')
        ui_file_main = os.path.join(rospkg.RosPack().get_path('tbf_gripper_rqt'), 'resource', 'RobotiqMain.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file_finger, self._wdg_fingerA, {'RobotiqFinger': RobotiqFinger})
        loadUi(ui_file_finger, self._wdg_fingerB, {'RobotiqFinger': RobotiqFinger})
        loadUi(ui_file_finger, self._wdg_fingerC, {'RobotiqFinger': RobotiqFinger})
        loadUi(ui_file_finger, self._wdg_fingerS, {'RobotiqFinger': RobotiqFinger})
        loadUi(ui_file_main, self._wdg_main, {'RobotiqMain': RobotiqHand})
        # Give QObjects reasonable names
        self._wdg_fingerA.setObjectName('RobotiqFingerA')
        self._wdg_fingerB.setObjectName('RobotiqFingerB')
        self._wdg_fingerC.setObjectName('RobotiqFingerC')
        self._wdg_fingerS.setObjectName('RobotiqFingerS')
        self._wdg_main.setObjectName('RobotiqMain')
        self._widget.setObjectName('RobtiqHand')

        # name the fingers in the GUI
        self._wdg_fingerA.lbl_fingerID.setText('Finger A')
        self._wdg_fingerB.lbl_fingerID.setText('Finger B')
        self._wdg_fingerC.lbl_fingerID.setText('Finger C')
        self._wdg_fingerS.lbl_fingerID.setText('Finger S')

        #setup buttongroup for the grasping mode
        bg_rMOD = QtGui.QButtonGroup(self._wdg_main)
        bg_rMOD.addButton(self._wdg_main.rb_basic)
        bg_rMOD.addButton(self._wdg_main.rb_pinch)
        bg_rMOD.addButton(self._wdg_main.rb_wide)
        bg_rMOD.addButton(self._wdg_main.rb_scissor)
        bg_rMOD.setId(self._wdg_main.rb_basic, 0)
        bg_rMOD.setId(self._wdg_main.rb_pinch, 1)
        bg_rMOD.setId(self._wdg_main.rb_wide, 2)
        bg_rMOD.setId(self._wdg_main.rb_scissor, 3)

        mainLayout = QtGui.QVBoxLayout(self._widget)
        fingerLayout = QtGui.QHBoxLayout()

        fingerLayout.addWidget(self._wdg_fingerA)
        fingerLayout.addWidget(self._wdg_fingerB)
        fingerLayout.addWidget(self._wdg_fingerC)
        fingerLayout.addWidget(self._wdg_fingerS)

        mainLayout.addLayout(fingerLayout)
        mainLayout.addWidget(self._wdg_main)

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
        self.model = RobotiqHandModel()

        # connect signal and slots
        self._connectQtAndFingerModel(self._wdg_fingerA, self.model.mdl_fingerA)
        self._connectQtAndFingerModel(self._wdg_fingerB, self.model.mdl_fingerB)
        self._connectQtAndFingerModel(self._wdg_fingerC, self.model.mdl_fingerC)
        self._connectQtAndFingerModel(self._wdg_fingerS, self.model.mdl_fingerS)
        self._connectQtAndHandModel(self._wdg_main, self.model)
        self._widget.connect(bg_rMOD, QtCore.SIGNAL('buttonClicked(int)'), self.model.onMODChanged)

        #add intersction logic
        self._wdg_main.chk_rACT.stateChanged.connect(self._wdg_fingerA.setEnabled)
        self._wdg_main.chk_rICF.stateChanged.connect(self._wdg_fingerB.setEnabled)
        self._wdg_main.chk_rICF.stateChanged.connect(self._wdg_fingerC.setEnabled)

        self._wdg_main.chk_rICS.stateChanged.connect(self._wdg_fingerS.setEnabled)

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

    def _connectQtAndFingerModel(self, widget, model):
        widget.sb_rFR.valueChanged.connect(model.onForceRequestChanged)
        widget.sb_rPR.valueChanged.connect(model.onPositionRequestChanged)
        widget.sb_rSP.valueChanged.connect(model.onSpeedRequestChanged)
        model.atNewPR.connect(widget.lcd_gPR.display)
        model.atNewCU.connect(widget.lcd_gCU.display)
        model.atNewDT.connect(widget.lbl_gDT_value.setText)
        model.atNewPO.connect(widget.lcd_gPO.display)

    def _connectQtAndHandModel(self, widget, model):
        widget.btn_rATR.clicked.connect(model.onReleaseClicked)
        widget.chk_rGTO.stateChanged.connect(model.onGoToChanged)
        widget.chk_rICF.stateChanged.connect(model.onIndividualFingerChanged)
        widget.chk_rACT.stateChanged.connect(model.onActivationChanged)
        widget.chk_rGLV.stateChanged.connect(model.onGloveChanged)
        widget.chk_rICS.stateChanged.connect(model.onIndividualScissorChanged)
        model.atNewACT.connect(widget.chk_gACT.setChecked)
        model.atNewMOD.connect(widget.lbl_gMOD.setText)
        model.atNewGTO.connect(widget.chk_gGTO.setChecked)
        model.atNewIMC.connect(widget.lbl_gIMC.setText)
        model.atNewSTA.connect(widget.lbl_gSTA.setText)


class RobotiqHandModel(QtCore.QObject):
    """
    A instance of this class holds the data model for the Robotiq gripper.
    It holds the necessary signals and slots to communicate with the RobotiqHand-GUI. The RobotiqHand-controller
    establishes the connection.
    """
    atNewACT = QtCore.Signal(bool)  # Initialization status, echo of the rACT bit (activation bit).
    atNewMOD = QtCore.Signal(str)   # Operation Mode status, echo of the rMOD bits (grasping mode requested).
    atNewGTO = QtCore.Signal(bool)  # Action status, echo of the rGTO bit (go to bit).
    atNewIMC = QtCore.Signal(str)   # Gripper status, returns the current status of the Gripper.
    atNewSTA = QtCore.Signal(str)   # Gripper status, returns the current status & motion of the Gripper fingers.

    def __init__(self):
        """
        Default constructor - initializes the data model of the gripper, setting all register to 0, 4 finger models and
        subscribing to 'SModelRobotInput' as well as publishing messages via 'SModelRobotOutput'
        @return:
        """
        super(RobotiqHandModel, self).__init__()

        self.rMOD = 0
        self.rATR = 0
        self.rGTO = 0
        self.rICF = 0
        self.rACT = 0
        self.rGLV = 0
        self.rICS = 0

        self.gMOD = 0
        self.gACT = 0
        self.gGTO = 0
        self.gIMC = 0
        self.gSTA = 0

        self.mdl_fingerA = RobotiqFingerModel('A', self)
        self.mdl_fingerB = RobotiqFingerModel('B', self)
        self.mdl_fingerC = RobotiqFingerModel('C', self)
        self.mdl_fingerS = RobotiqFingerModel('S', self)

        self.subscriber = rospy.Subscriber("SModelRobotInput", inputMsg, self.onReceivedROSMessage)
        self.publisher = rospy.Publisher('SModelRobotOutput', outputMsg, queue_size=10)

    @QtCore.Slot(int)
    def onActivationChanged(self, obj):
        """
        First action to be made prior to any other actions, rACT bit will initialize the Adaptive Gripper. Clear rACT to
        reset the Gripper and clear fault status.
        @param obj: refers to rACT
        @return: None
        """
        # rospy.logwarn("hand_module.py@RobotiqHandModel.onActivationChanged(): newValue="+str(obj)+" type:"+str(type(obj)))
        # Checkbox has 3 states (0=unchecked, 1=not specified, 2=checked)
        self.rACT = int(obj != 0)
        self.sendROSMessage()

    @QtCore.Slot(int)
    def onMODChanged(self, obj):
        """
        Changes the Gripper Grasping Mode. When the Grasping Mode is changed, the Gripper first opens completely
        to avoid interference between the fingers, then goes to the selected mode. This option is ignored if the bit
        rICS is set (individual control of the scissor motion option).
        @param obj: refers to rMOD
        @return: None
        """
        # rospy.logwarn("hand_module.py@RobotiqHandModel.onMODChanged(): newValue="+str(obj)+" type:"+str(type(obj)))
        self.rMOD = obj
        self.sendROSMessage()

    @QtCore.Slot(int)
    def onGoToChanged(self, obj):
        """
        The "Go To" action moves the Gripper fingers to the requested position using the configuration defined by the
        other registers and the rMOD bits. The only motions performed without the rGTO bit are: activation, the mode
        change and automatic release routines.
        @param obj: refers to rGTO
        @return: None
        """
        # rospy.logwarn("hand_module.py@RobotiqHandModel.onGoToChanged(): newValue="+str(obj)+" type:"+str(type(obj)))
        self.rGTO = int(obj != 0)
        self.sendROSMessage()

    @QtCore.Slot(int)
    def onIndividualFingerChanged(self, obj):
        """
        In Individual Control of Fingers Mode each finger receives its own command (position request, speed and force)
        unless the Gripper is in the Scissor Grasping Mode and the Independent Control of Scissor (rICS) is not
        activated. Please refer to the rPRA (Position Request) register description for information about the reachable
        positions of the fingers.
        @param obj: refers to rICF
        @return: None
        """
        # rospy.logwarn("hand_module.py@RobotiqHandModel.onIndividualFingerChanged(): newValue="+str(obj)+" type:"+str(type(obj)))
        self.rICF = int(obj != 0)
        self.sendROSMessage()

    @QtCore.Slot(int)
    def onIndividualScissorChanged(self, obj):
        """
        In Individual Control of Scissor, the scissor axis moves independently from the Grasping Mode. When this option
        is selected, the rMOD bits (Grasping Mode) are ignored as the scissor axis position is defined by the rPRS
        (Position Request for the Scissor axis) register which takes priority.
        @param obj: refers to rICS
        @return: None
        """
        # rospy.logwarn("hand_module.py@RobotiqHandModel.onIndividualScissorChanged(): newValue="+str(obj)+" type:"+str(type(obj)))
        self.rICS = int(obj != 0)
        self.sendROSMessage()

    @QtCore.Slot(int)
    def onReleaseClicked(self, obj):
        """
        Automatic Release routine action slowly opens the Gripper fingers until all motion axes reach their mechanical
        limits. After all motion is completed, the Gripper sends a fault signal and needs to be reinitialized before any
        other motion is performed. The rATR bit overrides all other commands excluding the activation bit (rACT).
        @param obj: not used
        @return: None
        """
        # rospy.logwarn("hand_module.py@RobotiqHandModel.onReleaseClicked(): newValue="+str(obj)+" type:"+str(type(obj)))
        self.rATR = 1
        self.sendROSMessage()
        self.rATR = 0

    @QtCore.Slot(int)
    def onGloveChanged(self, obj):
        """
        The Glove Mode option must be on when using the Robotiq Glove on the Adaptive Gripper. Using the Robotiq Glove
        without this option could result in a hazardous behavior from the Gripper. The glove mode option will change the
        control of your 3-Finger Gripper. For example, fingers A and B will have a wider opening to avoid friction
        between fingers due to the glove and minimum force is raised to counter the glove resistance.
        @param obj: refers to rGLV
        @return: None
        """
        # rospy.logwarn("hand_module.py@RobotiqHandModel.onGloveChanged(): newValue="+str(obj)+" type:"+str(type(obj)))
        self.rGLV = int(obj != 0)
        self.sendROSMessage()

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
        msg.rPRB = self.mdl_fingerB.rPR
        msg.rSPB = self.mdl_fingerB.rSP
        msg.rFRB = self.mdl_fingerB.rFR
        msg.rPRC = self.mdl_fingerC.rPR
        msg.rSPC = self.mdl_fingerC.rSP
        msg.rFRC = self.mdl_fingerC.rFR
        msg.rPRS = self.mdl_fingerS.rPR
        msg.rSPS = self.mdl_fingerS.rSP
        msg.rFRS = self.mdl_fingerS.rFR
        # rospy.logwarn("hand_module.py@RobotiqHandModel.sendROSMessage(): msg="+str(msg))
        self.publisher.publish(msg)

    def onReceivedROSMessage(self, data):
        """
        callback for the ROS subscriber
        after receiving a ROS message the registers of the hand are set and the GUI, as well as fingers, is updated
        @param data: ROS message of the type 'SModelRobotInput'
        @return: None
        """
        # rospy.logwarn("hand_module.py@RobotiqFingerModel.onReceivedROSMessage(): received message=\n"+str(data))
        self.gMOD = data.gMOD
        self.gACT = data.gACT
        self.gGTO = data.gGTO
        self.gIMC = data.gIMC
        self.gSTA = data.gSTA
        self.afterReceivedROSMessage()

    def afterReceivedROSMessage(self):
        """
        update the GUI, or other listening classes, via Qt-signals
        @return: None
        """
        #update status
        self.atNewACT[bool].emit(self.gACT == 1)
        self.atNewIMC[str].emit(RobotiqHandModel._sendGripperState(self.gIMC))
        self.atNewSTA[str].emit(RobotiqHandModel._sendGripperMotion(self.gSTA))
        # self.atNewMOD[str].emit(RobotiqHandModel._sendMode(self.gMOD))
        # self.atNewMOD[str].emit("foobar")
        self.atNewGTO[bool].emit(self.gGTO == 1)

    def shutdown(self):
        """
        called when the model will be destroyed - unregister the publisher, if available, and destroy all sub-models
        @return: None
        """
        self.mdl_fingerA.shutdown()
        self.mdl_fingerB.shutdown()
        self.mdl_fingerC.shutdown()
        self.mdl_fingerS.shutdown()
        self.publisher.unregister()

    @staticmethod
    def _sendMode(gMOD):
        if gMOD == 0:
            return 'Basic Mode'
        elif gMOD == 1:
            return 'Pinch Mode'
        elif gMOD == 2:
            return 'Wide Mode'
        elif gMOD == 3:
            return 'Scissor Mode'
        else:
            rospy.logwarn("hand_module.py@RobotiqHandModel._sendMode(): Unknown gMOD: "+str(gMOD))
            return 'ERROR'

    @staticmethod
    def _sendGripperState(gIMC):
        if gIMC == 0:
            return 'Gripper is in reset'
        elif gIMC == 1:
            return 'Activation is in progress'
        elif gIMC == 2:
            return 'Mode change is in progress'
        elif gIMC == 3:
            return 'Activation and Mode change are complete'
        else:
            rospy.logwarn("hand_module.py@RobotiqHandModel._sendGripperState(): Unknown gIMC: "+str(gIMC))
            return 'ERROR'

    @staticmethod
    def _sendGripperMotion(gSTA):
        if gSTA == 0:
            return 'Gripper is in motion towards requested position'
        elif gSTA == 1:
            return 'Gripper is stopped. One or two fingers stopped before requested position.'
        elif gSTA == 2:
            return 'Gripper is stopped. All fingers stopped before requested position.'
        elif gSTA == 3:
            return 'Gripper is stopped. All fingers reached requested position.'
        else:
            rospy.logwarn("hand_module.py@RobotiqHandModel._sendGripperMotion(): Unknown gSTA: "+str(gSTA))
            return 'ERROR'
