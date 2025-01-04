package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConfigConstants;
import org.firstinspires.ftc.teamcode.Constants;

public class Lift extends SubsystemBase{
    private Motor leftMotor;
    private Motor rightMotor;
    private AnalogInput absoluteEncoder;

    private Sensors mSensors;
    private Telemetry mTelemetry;

    public double minPositionCM = ConfigConstants.Lift.MIN_POS_CM;
    public double maxPositionCM = ConfigConstants.Lift.MAX_POS_CM;

    private boolean pidEnabled = false;
    private double pidTarget = minPositionCM;
    private double manualPower = 0.0;

    private PIDController liftUpPID;
    private PIDController liftDownPID;
    private Constants.Lift.MovementDirection movementDirectionForPID = Constants.Lift.MovementDirection.NONE;

    public Lift(HardwareMap hardwareMap, Sensors sensors, Telemetry telemetry){
        leftMotor = new Motor(hardwareMap, Constants.HardwareMapping.liftLeftMotor);
        rightMotor = new Motor(hardwareMap, Constants.HardwareMapping.liftRightMotor);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, Constants.HardwareMapping.liftAbsoluteEncoder);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        mSensors = sensors;
        mTelemetry = telemetry;

        liftUpPID = new PIDController(
                ConfigConstants.Lift.LiftPID.kPUp,
                ConfigConstants.Lift.LiftPID.kIUp,
                ConfigConstants.Lift.LiftPID.kDUp);
        liftUpPID.setTolerance(ConfigConstants.Lift.LIFT_TOLERANCE_UP);
        liftUpPID.setSetPoint(ConfigConstants.Lift.MIN_POS_CM + ConfigConstants.Lift.LIFT_TOLERANCE_UP); // Slightly above hard stop
        liftDownPID = new PIDController(
                ConfigConstants.Lift.LiftPID.kPDown,
                ConfigConstants.Lift.LiftPID.kIDown,
                ConfigConstants.Lift.LiftPID.kDDown);
        liftDownPID.setTolerance(ConfigConstants.Lift.LIFT_TOLERANCE_DOWN);
        liftDownPID.setSetPoint(ConfigConstants.Lift.MIN_POS_CM + ConfigConstants.Lift.LIFT_TOLERANCE_DOWN); // Slightly above hard stop
    }

    @Override
    public void periodic() {
        if (pidEnabled) {
            double power = 0.0;
            // Power is adjusted by a constant feed forward to account for gravity
            double upPower = liftUpPID.calculate(getPositionCM(), pidTarget) + ConfigConstants.Lift.LiftPID.kFUp;
            double downPower = liftDownPID.calculate(getPositionCM(), pidTarget) + ConfigConstants.Lift.LiftPID.kFDown;
            switch (movementDirectionForPID) {
                case NONE:
                    // THIS SHOULD NOT HAPPEN.
                    power = 0.0;
                    break;
                case UP:
                    // Going up
                    power = upPower;
                    break;
                case DOWN:
                    // Going down
                    power = downPower;
            }
            // Clamp max PID speed
            if (getPositionCM() < ConfigConstants.Lift.MIN_POS_CM + ConfigConstants.Lift.HOME_SLOW_ZONE_THRESHOLD) {
                power = Math.min(power, ConfigConstants.Lift.MAX_UP_POWER);
                power = Math.max(power, ConfigConstants.Lift.MAX_DOWN_POWER_CLOSE);
            } else if (
                    (getPositionCM() < pidTarget + ConfigConstants.Lift.TARGET_SLOW_ZONE_THRESHOLD) &&
                    (getPositionCM() > pidTarget - ConfigConstants.Lift.TARGET_SLOW_ZONE_THRESHOLD)
            ) {
                power = Math.min(power, ConfigConstants.Lift.MAX_UP_POWER_CLOSE);
                power = Math.max(power, ConfigConstants.Lift.MAX_DOWN_POWER_CLOSE);
            } else {
                power = Math.min(power, ConfigConstants.Lift.MAX_UP_POWER);
                power = Math.max(power, ConfigConstants.Lift.MAX_DOWN_POWER);
            }
            leftMotor.set(power);
            rightMotor.set(power);
        } else {
            liftUpPID.reset();
            liftDownPID.reset();
            leftMotor.set(manualPower);
            rightMotor.set(manualPower);
        }

        mTelemetry.addData("Lift: Pos cm", getPositionCM());
        mTelemetry.addData("Lift: Abs Enc V", absoluteEncoder.getVoltage());
        mTelemetry.addData("Lift: L Enc", mSensors.liftLeftEncoderPosition);
        mTelemetry.addData("Lift: R Enc", mSensors.liftRightEncoderPosition);
        mTelemetry.addData("Lift: PID Enabled", pidEnabled);
        mTelemetry.addData("Lift: PID target", pidTarget);
        mTelemetry.addData("Lift: Manual Power", manualPower);
    }

    public double getPositionCM(){
        return (absoluteEncoder.getVoltage() - Constants.Lift.MIN_V) * Constants.Lift.LIFT_V_TO_CM;
    }

    public void setMotorPower(double power ){
        if (power > 1.0){ power = 1.0;}
        if (power < -1.0){ power = -1.0;}

        pidEnabled = false;
        movementDirectionForPID = Constants.Lift.MovementDirection.NONE;
        manualPower = power;
    }

    public void stopMotors() {
        pidEnabled = false;
        movementDirectionForPID = Constants.Lift.MovementDirection.NONE;
        manualPower = 0.0;
    }

    /**
     *
     * @param target height in CM
     *
     * Enforces min and max range on target height
     *
     */
    public void setPidTarget (double target) {
        double safeHeight = Math.min(target, ConfigConstants.Lift.MAX_POS_CM);
        safeHeight = Math.max(safeHeight, ConfigConstants.Lift.MIN_POS_CM);
        this.pidTarget = safeHeight;
        liftUpPID.setSetPoint(safeHeight);
        liftDownPID.setSetPoint(safeHeight);
    }

    /**
     *
     * @return height target in CM
     */
    public double getPidTarget() {
        return this.pidTarget;
    }

    public void disablePID() {
        this.pidEnabled = false;
        this.movementDirectionForPID = Constants.Lift.MovementDirection.NONE;
    }

    /**
     *
     * @return Whether or not PID is currently running
     *
     */
    public boolean isPidEnabled() {
        return pidEnabled;
    }

    /**
     *
     * @param height -- Enable the Lift PID and set the height target in CM
     */
    public void setHeight(double height) {
        setPidTarget(height);
        this.pidEnabled = true;
        if (getPositionCM() < height) {
            movementDirectionForPID = Constants.Lift.MovementDirection.UP;
        } else {
            movementDirectionForPID = Constants.Lift.MovementDirection.DOWN;
        }
    }
}
