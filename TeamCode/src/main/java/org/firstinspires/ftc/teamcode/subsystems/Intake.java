package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.ContinuousServo;
import org.firstinspires.ftc.teamcode.lib.GamePieceColor;

public class Intake extends SubsystemBase{
    // Discrete Servos
    private ServoEx pivotLeft;
    private ServoEx pivotRight;
    private ServoEx slideLeft;
    private ServoEx slideRight;
    // Continuous Servos
    private ContinuousServo intakeLeft;
    private ContinuousServo intakeRight;

//    private SensorColor colorSensor;
    private SensorRevTOFDistance distanceSensor;
    private NormalizedColorSensor colorSensor;

    private Telemetry mTelemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        intakeLeft = new ContinuousServo(hardwareMap, Constants.HardwareMapping.intakeContinuousLeftServo);
        intakeRight = new ContinuousServo(hardwareMap, Constants.HardwareMapping.intakeContinuousRightServo);
        intakeLeft.setInverted(false);
        intakeRight.setInverted(true);

        pivotLeft = new SimpleServo(hardwareMap,
                Constants.HardwareMapping.intakePivotLeftServo,
                Constants.Intake.PIVOT_MIN_ANGLE_LEFT_DEG,
                Constants.Intake.PIVOT_MAX_ANGLE_LEFT_DEG,
                AngleUnit.DEGREES);
        pivotRight = new SimpleServo(hardwareMap,
                Constants.HardwareMapping.intakePivotLeftServo,
                Constants.Intake.PIVOT_MIN_ANGLE_RIGHT_DEG,
                Constants.Intake.PIVOT_MAX_ANGLE_RIGHT_DEG,
                AngleUnit.DEGREES);

        slideLeft = new SimpleServo(hardwareMap,
                Constants.HardwareMapping.intakePivotLeftServo,
                Constants.Intake.SLIDE_MIN_ANGLE_LEFT_DEG,
                Constants.Intake.SLIDE_MAX_ANGLE_LEFT_DEG,
                AngleUnit.DEGREES);
        slideRight = new SimpleServo(hardwareMap,
                Constants.HardwareMapping.intakePivotLeftServo,
                Constants.Intake.SLIDE_MIN_ANGLE_RIGHT_DEG,
                Constants.Intake.SLIDE_MAX_ANGLE_RIGHT_DEG,
                AngleUnit.DEGREES);

        slideLeft = new SimpleServo(hardwareMap,
                Constants.HardwareMapping.intakePivotLeftServo,
                Constants.Intake.SLIDE_MIN_ANGLE_LEFT_DEG,
                Constants.Intake.SLIDE_MAX_ANGLE_LEFT_DEG,
                AngleUnit.DEGREES);
        slideRight = new SimpleServo(hardwareMap,
                Constants.HardwareMapping.intakePivotLeftServo,
                Constants.Intake.SLIDE_MIN_ANGLE_RIGHT_DEG,
                Constants.Intake.SLIDE_MAX_ANGLE_RIGHT_DEG,
                AngleUnit.DEGREES);

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
        int[] argbArr = {(int)(argb.alpha * 255), (int)(argb.red * 255), (int)(argb.green * 255), (int)(argb.blue * 255)};
//        mTelemetry.addData("Intake: L Pivot Pos", pivotLeft.getPosition());
//        mTelemetry.addData("Intake: R Pivot Pos", pivotRight.getPosition());
//        mTelemetry.addData("Intake: L Slide Pos", slideLeft.getPosition());
//        mTelemetry.addData("Intake: R Slide Pos", slideRight.getPosition());
        mTelemetry.addData("Intake: L Cont Power", intakeLeft.getPower());
        mTelemetry.addData("Intake: R Cont Power", intakeRight.getPower());
        mTelemetry.addData("Intake: RED Match", GamePieceColor.RED.matches(hsv[0]));
        mTelemetry.addData("Intake: BLUE Match", GamePieceColor.BLUE.matches(hsv[0])    );
        mTelemetry.addData("Intake: YELLOW Match", GamePieceColor.YELLOW.matches(hsv[0]));
        mTelemetry.addData("Intake: BLACK Match", GamePieceColor.BLACK.matches(hsv[0]));
        mTelemetry.addData("Intake: Color Sensor Dist (cm)", distanceSensor.getDistance(DistanceUnit.CM));
        mTelemetry.addData("Intake: Color h", "%.3f", hsv[0]);
    }

    public void intakePiece() {
        intakeLeft.reverse();
        intakeRight.reverse();
    }

    public void outtakePiece() {
        intakeLeft.forward();
        intakeRight.forward();
    }

    public void stopIntake() {
        intakeLeft.stop();
        intakeRight.stop();
    }

    public void pivotToPosition(Constants.Intake.PivotSetPosition position) {
        pivotLeft.setPosition(position.leftPosition);
        pivotRight.setPosition(position.rightPosition);
    }

    public void slideToPosition(Constants.Intake.SlideSetPosition position) {
        slideLeft.setPosition(position.leftPosition);
        slideRight.setPosition(position.rightPosition);
    }

    private NormalizedRGBA getColor() {
        return colorSensor.getNormalizedColors();
    }
}
