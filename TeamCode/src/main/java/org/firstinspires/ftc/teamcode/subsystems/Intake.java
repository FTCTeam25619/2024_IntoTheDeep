package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.ContinuousServo;
import org.firstinspires.ftc.teamcode.lib.GamePieceColor;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase{
    // Discrete Servos
    private Servo pivotLeft;
    private Servo pivotRight;
    private Servo slideLeft;
    private Servo slideRight;
    // Continuous Servos
    private ContinuousServo intakeLeft;
    private ContinuousServo intakeRight;

    private SensorRevTOFDistance distanceSensor;
    private NormalizedColorSensor colorSensor;

    private Telemetry mTelemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        intakeLeft = new ContinuousServo(hardwareMap, Constants.HardwareMapping.intakeContinuousLeftServo);
        intakeRight = new ContinuousServo(hardwareMap, Constants.HardwareMapping.intakeContinuousRightServo);
        intakeLeft.setInverted(false);
        intakeRight.setInverted(true);

        pivotLeft = hardwareMap.get(Servo.class, Constants.HardwareMapping.intakePivotLeftServo);
        pivotRight = hardwareMap.get(Servo.class, Constants.HardwareMapping.intakePivotRightServo);

        slideLeft = hardwareMap.get(Servo.class, Constants.HardwareMapping.intakeSlideLeftServo);
        slideRight = hardwareMap.get(Servo.class, Constants.HardwareMapping.intakeSlideRightServo);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, Constants.HardwareMapping.intakeColorSensor);
        distanceSensor = new SensorRevTOFDistance(hardwareMap, Constants.HardwareMapping.intakeColorSensor);
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            NormalizedRGBA argb = getColor();
            float[] hsv = new float[3];
            Color.colorToHSV(argb.toColor(), hsv);
            float hue = hsv[0];
            mTelemetry.addData("Intake: L Pivot Pos", pivotLeft.getPosition());
            mTelemetry.addData("Intake: R Pivot Pos", pivotRight.getPosition());
            mTelemetry.addData("Intake: L Slide Pos", slideLeft.getPosition());
            mTelemetry.addData("Intake: R Slide Pos", slideRight.getPosition());
            mTelemetry.addData("Intake: L Cont Power", intakeLeft.getPower());
            mTelemetry.addData("Intake: R Cont Power", intakeRight.getPower());
            mTelemetry.addData("Intake: Color h", "%.3f", hue);
            mTelemetry.addData("Intake: RED Match", GamePieceColor.RED.matches(hue));
            mTelemetry.addData("Intake: BLUE Match", GamePieceColor.BLUE.matches(hue));
            mTelemetry.addData("Intake: YELLOW Match", GamePieceColor.YELLOW.matches(hue));
            mTelemetry.addData("Intake: BLACK Match", GamePieceColor.BLACK.matches(hue));
            mTelemetry.addData("Intake: Color Sensor Dist (cm)", distanceSensor.getDistance(DistanceUnit.CM));
        }
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

    /**
     * Set the intake slide to a particular position.  Input is left servo target position.
     *
     * Assumes that the OUT leftPosition is < IN leftPosition.
     *
     * @param leftSlidePosition
     */
    public void slideToManualPosition(double leftSlidePosition) {
        double leftTarget = MathUtils.clamp(leftSlidePosition,
                Constants.Intake.SlideSetPosition.OUT_NEAR.leftPosition,
                Constants.Intake.SlideSetPosition.OUT_FAR.leftPosition);
        double rightTarget = matchingRightSlidePoint(leftTarget);
        mTelemetry.addData("Intake: slide manual L", leftTarget);
        mTelemetry.addData("Intake: slide manual R:",rightTarget);
        slideLeft.setPosition(leftTarget);
        slideRight.setPosition(rightTarget);
    }

    public void moveSlideManual(DoubleSupplier speedSupplier, double increment) {
        double speed = speedSupplier.getAsDouble();
        // Deadband
        if (speed < ConfigConstants.ManualMovement.slideManualDeadband) { return; }
        double smoothedSpeed = speed * speed * speed * increment;
        double currentPosition = slideLeft.getPosition();
        mTelemetry.addData("Intake: slide inc", smoothedSpeed);
        double newPosition = currentPosition + smoothedSpeed;
        newPosition = MathUtils.clamp(newPosition,
                Constants.Intake.SlideSetPosition.OUT_NEAR.leftPosition,
                Constants.Intake.SlideSetPosition.OUT_FAR.leftPosition);
        mTelemetry.addData("Intake: slide moved", newPosition - currentPosition);
        slideToManualPosition(newPosition);
    }

    /**
     *
     * Uses the OUT_NEAR, OUT_FAR enum constants and assumes linearity in converting slide Left servo
     * setpoint to the matching right servo setpoint
     *
     * @param leftSlidePoint The Left slide servo position
     * @return returns the matching value to use for right slide servo
     */
    private double matchingRightSlidePoint (double leftSlidePoint) {
        return (leftSlidePoint - Constants.Intake.SlideSetPosition.OUT_FAR.leftPosition) *
                Constants.Intake.rightToLeftSlideRatio +
                Constants.Intake.SlideSetPosition.OUT_FAR.rightPosition;
    }

    public void pivotLeftToTestPosition() {
        pivotLeft.setPosition(ConfigConstants.TestPositions.pivotLeftTest);
    }

    public void pivotRightToTestPosition() {
        pivotRight.setPosition(ConfigConstants.TestPositions.pivotRightTest);
    }

    public void slideLeftToTestPosition() {
        slideLeft.setPosition(ConfigConstants.TestPositions.slideLeftTest);
    }

    public void slideRightToTestPosition() {
        slideRight.setPosition(ConfigConstants.TestPositions.slideRightTest);
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
