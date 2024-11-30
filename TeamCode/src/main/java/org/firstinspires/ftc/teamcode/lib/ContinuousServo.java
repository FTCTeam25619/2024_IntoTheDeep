package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import kotlin.coroutines.ContinuationInterceptor;

public class ContinuousServo {
    private final CRServo servo;

    public static enum ContinuousServoDirection {
        REVERSE(0.0),
        STOP(0.5),
        FORWARD(1.0);

        public final double power;

        private ContinuousServoDirection(double pwr) {
            this.power = pwr;
        }
    }

    public ContinuousServo(HardwareMap hw, String servoName) {
        servo = hw.get(CRServo.class, servoName);
    }

    public void move(ContinuousServoDirection direction) {
        servo.setPower(direction.power);
    }

    public void stop() {
        servo.setPower(ContinuousServoDirection.STOP.power);
    }

    public void forward() {
        servo.setPower(ContinuousServoDirection.FORWARD.power);
    }

    public void reverse() {
        servo.setPower(ContinuousServoDirection.REVERSE.power);
    }

    public void setInverted(boolean isInverted) {
        servo.setDirection(isInverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public boolean getInverted() {
        return DcMotorSimple.Direction.REVERSE == servo.getDirection();
    }

    public double getPower() {
        return servo.getPower();
    }

    public ContinuousServoDirection getCurrentMovement() {
        double currentPower = servo.getPower();
        if (currentPower == ContinuousServoDirection.REVERSE.power) { return ContinuousServoDirection.REVERSE; }
        if (currentPower == ContinuousServoDirection.FORWARD.power) { return ContinuousServoDirection.FORWARD; }
        return ContinuousServoDirection.STOP;
    }

    public void disable() {
        servo.close();
    }

    public String getDeviceType() {
        String port = Integer.toString(servo.getPortNumber());
        String controller = servo.getController().toString();
        return "ContinuousServo: " + port + "; " + controller;
    }
}