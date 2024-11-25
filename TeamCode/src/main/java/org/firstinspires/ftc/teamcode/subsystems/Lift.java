package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
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

    // TODO: This is currently volts -- should actually be cm.
    public double minPositionCM = 1.5;
    public double maxPositionCM = 3.0;

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
    }

    @Override
    public void periodic() {
        mTelemetry.addData("Lift: Pos cm", getPositionCM());
        mTelemetry.addData("Lift: L Enc", mSensors.getLiftLeftEncoderCount());
        mTelemetry.addData("Lift: R Enc", mSensors.getLiftRightEncoderCount());
    }

    public double getPositionCM(){
        return absoluteEncoder.getVoltage() * Constants.ConversionFactors.LIFT_VOLTAGE_TO_CM;
    }

    public void setMotorPower(double power ){
        if (power > 1.0){ power = 1.0;}
        if (power < -1.0){ power = -1.0;}

        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void stopMotors() {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }
}
