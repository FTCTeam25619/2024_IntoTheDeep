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

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, Constants.HardwareMapping.depositorColorSensor);
        distanceSensor = new SensorRevTOFDistance(hardwareMap, Constants.HardwareMapping.depositorColorSensor);
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        mTelemetry = telemetry;
    }

    private void initializePositions() {
        gripToPosition(Constants.Depositor.GripSetPosition.OPEN);
        wristToPosition(Constants.Depositor.WristSetPosition.INTAKE);
        armToPosition(Constants.Depositor.ArmSetPosition.HOME);
    }

    @Override
    public void periodic() {
        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            float hue = getHue();
            boolean piece = seeingPiece();
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
            mTelemetry.addData("Depositor: Piece?", piece);
        }
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

    public void gripToTestPosition() {
        grip.setPosition(ConfigConstants.TestPositions.gripTest);
    }

    public void wristToTestPosition() {
        wrist.setPosition(ConfigConstants.TestPositions.wristTest);
    }

    public void awaitPiece() {
        if (seeingPiece()) {
            gripToPosition(Constants.Depositor.GripSetPosition.CLOSED);
        } else {
            gripToPosition(Constants.Depositor.GripSetPosition.OPEN);
        }
    }

    public void wristToPosition(Constants.Depositor.WristSetPosition position) {
        wrist.setPosition(position.position);
    }

    public void gripToPosition(Constants.Depositor.GripSetPosition position) {
        grip.setPosition(position.position);
    }

    private NormalizedRGBA getColor() {
        return colorSensor.getNormalizedColors();
    }

    private float getHue() {
        NormalizedRGBA argb = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(argb.toColor(), hsv);
        return hsv[0];
    }

    public boolean seeingPiece() {
        float hue = getHue();
        return GamePieceColor.BLUE.matches(hue) ||
                GamePieceColor.RED.matches(hue) ||
                GamePieceColor.YELLOW.matches(hue);
    }

    public GamePieceColor pieceColorSeen() {
        float hue = getHue();
        if (GamePieceColor.BLUE.matches(hue)) return GamePieceColor.BLUE;
        if (GamePieceColor.RED.matches(hue)) return GamePieceColor.RED;
        if (GamePieceColor.YELLOW.matches(hue)) return GamePieceColor.YELLOW;
        return GamePieceColor.BLACK;
    }
}
