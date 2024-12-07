package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.GamePieceColor;

public class Depositor extends SubsystemBase {
    private final Servo armLeft;
    private final Servo armRight;
    private final Servo wrist;
    private final Servo grip;
    private final NormalizedColorSensor colorSensor;
    private final SensorRevTOFDistance distanceSensor;
    private final Telemetry mTelemetry;

    public Depositor(HardwareMap hardwareMap, Telemetry telemetry) {
        armLeft = hardwareMap.get(Servo.class, Constants.HardwareMapping.depositorArmLeftServo);
        armRight = hardwareMap.get(Servo.class, Constants.HardwareMapping.depositorArmRightServo);
        grip = hardwareMap.get(Servo.class, Constants.HardwareMapping.depositorGripServo);
        wrist = hardwareMap.get(Servo.class, Constants.HardwareMapping.depositorWristServo);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, Constants.HardwareMapping.intakeColorSensor);
        distanceSensor = new SensorRevTOFDistance(hardwareMap, Constants.HardwareMapping.intakeColorSensor);
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        NormalizedRGBA argb = getColor();
        float[] hsv = new float[3];
        Color.colorToHSV(argb.toColor(), hsv);
        float hue = hsv[0];
        mTelemetry.addData("Depositor: L Arm Pos", armLeft.getPosition());
        mTelemetry.addData("Depositor: R Arm Pos", armRight.getPosition());
        mTelemetry.addData("Depositor: Wrist Pos", wrist.getPosition());
        mTelemetry.addData("Depositor: Grip Pos", grip.getPosition());
        mTelemetry.addData("Depositor: Color h", "%.3f", hue);
        mTelemetry.addData("Depositor: RED Match", GamePieceColor.RED.matches(hue));
        mTelemetry.addData("Depositor: BLUE Match", GamePieceColor.BLUE.matches(hue));
        mTelemetry.addData("Depositor: YELLOW Match", GamePieceColor.YELLOW.matches(hue));
        mTelemetry.addData("Depositor: BLACK Match", GamePieceColor.BLACK.matches(hue));
        mTelemetry.addData("Depositor: Color Sensor Dist (cm)", distanceSensor.getDistance(DistanceUnit.CM));
    }

    public void armToPosition(Constants.Depositor.ArmSetPosition position) {
        armLeft.setPosition(position.leftPosition);
        armRight.setPosition(position.rightPosition);
    }

    public void armLeftToTestPosition() {
        armLeft.setPosition(ConfigConstants.TestPositions.armLeftTest);
    }

    public void armRightToTestPosition() {
        armRight.setPosition(ConfigConstants.TestPositions.armRightTest);
    }

    public void wristToPosition(Constants.Depositor.WristSetPosition position) {
//        wrist.setPosition(position.positionDegrees);
        wrist.setPosition(ConfigConstants.TestPositions.wristTest);
    }

    public void gripToPosition(Constants.Depositor.GripSetPosition position) {
//        grip.setPosition(position.position);
        grip.setPosition(ConfigConstants.TestPositions.gripTest);
    }

    private NormalizedRGBA getColor() {
        return colorSensor.getNormalizedColors();
    }
}
