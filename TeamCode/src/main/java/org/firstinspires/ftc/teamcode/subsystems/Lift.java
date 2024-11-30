package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Lift extends SubsystemBase{
    private Motor leftMotor;
    private Motor rightMotor;
    private AnalogInput absoluteEncoder;

    private Sensors mSensors;
    private Telemetry mTelemetry;

    public double minPositionCM = Constants.Lift.MIN_POS_CM;
    public double maxPositionCM = Constants.Lift.MAX_POS_CM;

    private boolean pidEnabled = false;
    private double pidTarget = minPositionCM;
    private double manualPower = 0.0;

    private PIDController liftPID;

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

        liftPID = new PIDController(Constants.Lift.PID.kP, Constants.Lift.PID.kI, Constants.Lift.PID.kD);
        liftPID.setTolerance(Constants.Lift.LIFT_TOLERANCE);
        liftPID.setSetPoint(Constants.Lift.MIN_POS_CM + Constants.Lift.LIFT_TOLERANCE); // Slightly above hard stop
    }

    @Override
    public void periodic() {
        if (pidEnabled) {
            // Power is adjusted by a constant feed forward to account for gravity
            double power = liftPID.calculate(getPositionCM(), pidTarget) + Constants.Lift.PID.kF;
            // Clamp max PID speed
            power = Math.min(power, Constants.Lift.MAX_UP_POWER);
            power = Math.max(power, Constants.Lift.MAX_DOWN_POWER);
            leftMotor.set(power);
            rightMotor.set(power);
        } else {
            leftMotor.set(manualPower);
            rightMotor.set(manualPower);
        }

        mTelemetry.addData("Lift: Pos cm", getPositionCM());
        mTelemetry.addData("Lift: L Enc", mSensors.liftLeftEncoderPosition);
        mTelemetry.addData("Lift: R Enc", mSensors.liftRightEncoderPosition);
        mTelemetry.addData("Lift: PID Enabled", pidEnabled);
        mTelemetry.addData("Lift: PID target", pidTarget);
        mTelemetry.addData("Lift: Manual Power", manualPower);
    }

    public double getPositionCM(){
        return absoluteEncoder.getVoltage() * Constants.Lift.LIFT_V_TO_CM;
    }

    public void setMotorPower(double power ){
        if (power > 1.0){ power = 1.0;}
        if (power < -1.0){ power = -1.0;}

        pidEnabled = false;
        manualPower = power;
    }

    public void stopMotors() {
        pidEnabled = false;
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
        double safeHeight = Math.min(target, Constants.Lift.MAX_POS_CM);
        safeHeight = Math.max(safeHeight, Constants.Lift.MIN_POS_CM);
        this.pidTarget = safeHeight;
        liftPID.setSetPoint(safeHeight);
    }

    /**
     *
     * @return height target in CM
     */
    public double getPidTarget() {
        return this.pidTarget;
    }

    /**
     *
     * @param enabled -- Whether to enable PID processing
     *
     * Use in conjunction with setPidTarget(height) which should be called first.
     * As an alternative, you can call setHeight(height) which both sets the target and enables PID
     *
     */
    public void setPidEnabled (boolean enabled) {
        this.pidEnabled = enabled;
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
        setPidEnabled(true);
    }
}
