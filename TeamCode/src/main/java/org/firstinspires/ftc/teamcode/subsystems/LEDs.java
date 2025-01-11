package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.GamePieceColor;

public class LEDs extends SubsystemBase {
    private final RevBlinkinLedDriver ledDriver;
    private final Sensors mSensors;
    private final Intake mIntake;
    private final Depositor mDepositor;
    private final Telemetry mTelemetry;

    public LEDs(HardwareMap hw, Sensors sensors, Intake intake, Depositor depositor, Telemetry telemetry) {
        this.ledDriver = hw.get(RevBlinkinLedDriver.class, Constants.HardwareMapping.leds);
        this.mSensors = sensors;
        this.mIntake = intake;
        this.mDepositor = depositor;
        this.mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        GamePieceColor intakeColor = mIntake.pieceColorSeen();
        GamePieceColor depositorColor = mDepositor.pieceColorSeen();

        mTelemetry.addData("LEDs: Intake Blue", intakeColor == GamePieceColor.BLUE);
        mTelemetry.addData("LEDs: Intake Red", intakeColor == GamePieceColor.RED);
        mTelemetry.addData("LEDs: Intake Yellow", intakeColor == GamePieceColor.YELLOW);
        mTelemetry.addData("LEDs: Depositor Blue", depositorColor == GamePieceColor.BLUE);
        mTelemetry.addData("LEDs: Depositor Red", depositorColor == GamePieceColor.RED);
        mTelemetry.addData("LEDs: Depositor Yellow", depositorColor == GamePieceColor.YELLOW);

        if (intakeColor == GamePieceColor.RED) setRed();
        else if (intakeColor == GamePieceColor.BLUE) setBlue();
        else if (intakeColor == GamePieceColor.YELLOW) setYellow();
        else if (depositorColor == GamePieceColor.RED) setRedHeartbeat();
        else if (depositorColor == GamePieceColor.BLUE) setBlueHeartbeat();
        else if (depositorColor == GamePieceColor.YELLOW) setYellowHeartbeat();
        else setSinelonOcean();
    }

    public void setBpmOcean() {
        mTelemetry.addData("LEDs: Pattern", "BPM Ocean");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
    }

    public void setWavesOcean() {
        mTelemetry.addData("LEDs: Pattern", "Waves Ocean");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
    }

    public void setTwinklesOcean() {
        mTelemetry.addData("LEDs: Pattern", "Twinkles Ocean");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE);
    }

    public void setSinelonOcean() {
        mTelemetry.addData("LEDs: Pattern", "Sinelon Ocean");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
    }

    public void setRainbowOcean() {
        mTelemetry.addData("LEDs: Pattern", "Rainbow Ocean");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
    }

    public void setSilver() {
        mTelemetry.addData("LEDs: Pattern", "Chase Gray");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
    }

    public void setYellow() {
        mTelemetry.addData("LEDs: Pattern", "Yellow");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void setRed() {
        mTelemetry.addData("LEDs: Pattern", "Red");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void setBlue() {
        mTelemetry.addData("LEDs: Pattern", "Blue");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void setBlack() {
        mTelemetry.addData("LEDs: Pattern", "Black");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setYellowHeartbeat() {
        mTelemetry.addData("LEDs: Pattern", "Chase Yellow");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
    }

    public void setRedHeartbeat() {
        mTelemetry.addData("LEDs: Pattern", "Chase Red");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
    }

    public void setBlueHeartbeat() {
        mTelemetry.addData("LEDs: Pattern", "Chase Blue");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
    }

    public void setYellowChase() {
        mTelemetry.addData("LEDs: Pattern", "Chase Yellow");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
    }

    public void setRedChase() {
        mTelemetry.addData("LEDs: Pattern", "Chase Red");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
    }

    public void setBlueChase() {
        mTelemetry.addData("LEDs: Pattern", "Chase Blue");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.ledDriver.setPattern(pattern);
    }
}
