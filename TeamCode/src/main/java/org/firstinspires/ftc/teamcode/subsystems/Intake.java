package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    private SensorColor colorSensor;
    private SensorRevTOFDistance distanceSensor;

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

        colorSensor = new SensorColor(hardwareMap, Constants.HardwareMapping.intakeColorSensor);
        distanceSensor = new SensorRevTOFDistance(hardwareMap, Constants.HardwareMapping.intakeColorSensor);

        mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        int[] argb = getColorSensorARGB();
        mTelemetry.addData("Intake: L Pivot Pos", pivotLeft.getPosition());
        mTelemetry.addData("Intake: R Pivot Pos", pivotRight.getPosition());
        mTelemetry.addData("Intake: L Slide Pos", slideLeft.getPosition());
        mTelemetry.addData("Intake: R Slide Pos", slideRight.getPosition());
        mTelemetry.addData("Intake: L Cont Power", intakeLeft.getPower());
        mTelemetry.addData("Intake: R Cont Power", intakeRight.getPower());
        mTelemetry.addData("Intake: Color Sensor", argb);
        mTelemetry.addData("Intake: RED Match", GamePieceColor.RED.matches(argb));
        mTelemetry.addData("Intake: BLUE Match", GamePieceColor.BLUE.matches(argb));
        mTelemetry.addData("Intake: YELLOW Match", GamePieceColor.YELLOW.matches(argb));
        mTelemetry.addData("Intake: BLACK Match", GamePieceColor.BLACK.matches(argb));
        mTelemetry.addData("Intake: Color Sensor Dist (cm)", distanceSensor.getDistance(DistanceUnit.CM));
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

    private int[] getColorSensorARGB() {
        return colorSensor.getARGB();
    }
}
