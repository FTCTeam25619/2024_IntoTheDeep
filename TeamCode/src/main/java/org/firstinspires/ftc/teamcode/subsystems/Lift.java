package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpDriveSticks;

public class Lift extends SubsystemBase{
    private Motor leftMotor;
    private Motor rightMotor;
    private AnalogInput absoluteEncoder;

    private Telemetry mTelemetry;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry){
        leftMotor = new Motor(hardwareMap, Constants.HardwareMapping.liftLeftMotor);
        rightMotor = new Motor(hardwareMap, Constants.HardwareMapping.liftRightMotor);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, Constants.HardwareMapping.liftAbsoluteEncoder);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);



        mTelemetry = telemetry;


    }

    @Override
    public void periodic() {
        mTelemetry.addData("Lift: Position", getPositionCM());

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
}
