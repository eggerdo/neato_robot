#!/usr/bin/env python

# Generic driver for the Neato XV-11 Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
neato_driver.py is a generic driver for the Neato XV-11 Robotic Vacuum.
ROS Bindings can be found in the neato_node package.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import serial
import rospy

BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second

BRUSH_MAX_RPM = 500
VACUUM_MAX_SPEED = 100

xv11_analog_sensors = [ "WallSensorInMM",
                        "BatteryVoltageInmV",
                        "LeftDropInMM",
                        "RightDropInMM",
                        "LeftMagSensor",
                        "RightMagSensor",
                        "UIButtonInmV",
                        "VacuumCurrentInmA",
                        "ChargeVoltInmV",
                        "BatteryTemp0InC",
                        "BatteryTemp1InC",
                        "CurrentInmA",
                        "SideBrushCurrentInmA",
                        "VoltageReferenceInmV",
                        "AccelXInmG",
                        "AccelYInmG",
                        "AccelZInmG" ]

xv11_digital_sensors = [ "SNSR_DC_JACK_CONNECT",
                        "SNSR_DUSTBIN_IS_IN",
                        "SNSR_LEFT_WHEEL_EXTENDED",
                        "SNSR_RIGHT_WHEEL_EXTENDED",
                        "LSIDEBIT",
                        "LFRONTBIT",
                        "RSIDEBIT",
                        "RFRONTBIT" ]

xv11_motor_info = [ "Brush_RPM",
                    "Brush_mA",
                    "Vacuum_RPM",
                    "Vacuum_mA",
                    "LeftWheel_RPM",
                    "LeftWheel_Load",
                    "LeftWheel_PositionInMM",
                    "LeftWheel_Speed",
                    "RightWheel_RPM",
                    "RightWheel_Load",
                    "RightWheel_PositionInMM",
                    "RightWheel_Speed",
                    "Charger_mAH",
                    "SideBrush_mA" ]




xv11_charger_info = [ "FuelPercent",
                "BatteryOverTemp",
                "ChargingActive",
                "ChargingEnabled",
                "ConfidentOnFuel",
                "OnReservedFuel",
                "EmptyFuel",
                "BatteryFailure",
                "ExtPwrPresent",
                "ThermistorPresent[0]",
                "ThermistorPresent[1]",
                "BattTempCAvg[0]",
                "BattTempCAvg[1]",
                "VBattV",
                "VExtV",
                "Charger_mAH" ]

class xv11():

    def __init__(self, port="/dev/ttyUSB0"):
        self.port = serial.Serial(port,115200)
        # Storage for motor and sensor information
        self.state = {"LeftWheel_PositionInMM": 0, "RightWheel_PositionInMM": 0}
        self.stop_state = True
        # turn things on
        self.setTestMode("on")
        # self.setLDS("on")

    def exit(self):
        pass
        self.setLDS("off")
        self.setTestMode("off")

    def setTestMode(self, value):
        """ Turn test mode on/off. """
        self.port.write("testmode " + value + "\n")

    def setLDS(self, value):
        self.port.write("setldsrotation " + value + "\n")

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        self.port.flushInput()
        # rospy.loginfo("getldsscan")
        self.port.write("getldsscan\n")

    def getScanRanges(self):
        """ Read values of a scan -- call requestScan first! """
        ranges = list()
        angle = 0
        # rospy.loginfo("getScanRanges")
        try:
            line = self.port.readline()
            # rospy.loginfo("line: %s" %line)
        except:
            rospy.logerr("failed")
            return []
        while line.lstrip(str(unichr(26))).split(",")[0] != "AngleInDegrees":
            try:
                line = self.port.readline()
                # rospy.loginfo("line: %s" %line)
            except:
                rospy.logerr("failed")
                return []
        while angle < 360:
            try:
                vals = self.port.readline()
            except:
                rospy.logerr("failed")
                return []
            vals = vals.split(",")
            #print angle, vals
            # rospy.loginfo("angle: %d, value: %s" %(angle, vals))
            try:
                a = int(vals[0])
                r = int(vals[1])
                ranges.append(r/1000.0)
            except:
                ranges.append(0)
            angle += 1

        # rospy.loginfo(ranges)

        return ranges

    def setDriveMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then,
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
                l = 1
                r = 1
                s = 1
        else:
            self.stop_state = False

        # rospy.loginfo("setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s)))
        self.port.write("setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s))+"\n")

    def setBrush(self, speed):
        if (speed > 100):
            speed = 100

        rpm = int(BRUSH_MAX_RPM / 100 * speed)
        cmd = "RPM %d Brush" %rpm

        rospy.loginfo("setmotor RPM %d Brush" %rpm)
        self.port.write("setmotor RPM %d Brush\n" %rpm)

    def setVacuum(self, speed):
        if (speed > 100):
            speed = 100

        if speed == 0:
            cmd = "VacuumOff"
        else:
            cmd = "VacuumOn VacuumSpeed %d" %speed

        rospy.loginfo("setmotor %s" %cmd)
        self.port.write("setmotor %s\n" %cmd)

    def getMotors(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """
        self.port.flushInput()
        self.port.write("getmotors\n")
        line = self.port.readline()
        while line.split(",")[0] != "Parameter":
            try:
                line = self.port.readline()
            except:
                return [0,0]
        for i in range(len(xv11_motor_info)):
            try:
                values = self.port.readline().split(",")
                self.state[values[0]] = int(values[1])
            except:
                pass

        return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]]

    def getAnalogSensors(self):
        """ Update values for analog sensors in the self.state dictionary. """
        self.port.write("getanalogsensors\n")
        line = self.port.readline()
        while line.split(",")[0] != "SensorName":
            try:
                line = self.port.readline()
            except:
                return
        for i in range(len(xv11_analog_sensors)):
            try:
                values = self.port.readline().split(",")
                self.state[values[0]] = int(values[1])
            except:
                pass

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """
        self.port.write("getdigitalsensors\n")
        line = self.port.readline()
        while line.split(",")[0] != "Digital Sensor Name":
            try:
                line = self.port.readline()
            except:
                return
        for i in range(len(xv11_digital_sensors)):
            try:
                values = self.port.readline().split(",")
                self.state[values[0]] = int(values[1])
            except:
                pass

    def getCharger(self):
        """ Update values for charger/battery related info in self.state dictionary. """
        self.port.write("getcharger\n")
        line = self.port.readline()
        while line.split(",")[0] != "Label":
            line = self.port.readline()
        for i in range(len(xv11_charger_info)):
            values = self.port.readline().split(",")
            try:
                self.state[values[0]] = int(values[1])
            except:
                pass

    def setBacklight(self, value):
        if value > 0:
            self.port.write("setled backlighton")
        else:
            self.port.write("setled backlightoff")

    #SetLED - Sets the specified LED to on,off,blink, or dim. (TestMode Only)
    #BacklightOn - LCD Backlight On  (mutually exclusive of BacklightOff)
    #BacklightOff - LCD Backlight Off (mutually exclusive of BacklightOn)
    #ButtonAmber - Start Button Amber (mutually exclusive of other Button options)
    #ButtonGreen - Start Button Green (mutually exclusive of other Button options)
    #LEDRed - Start Red LED (mutually exclusive of other Button options)
    #LEDGreen - Start Green LED (mutually exclusive of other Button options)
    #ButtonAmberDim - Start Button Amber Dim (mutually exclusive of other Button options)
    #ButtonGreenDim - Start Button Green Dim (mutually exclusive of other Button options)
    #ButtonOff - Start Button Off

