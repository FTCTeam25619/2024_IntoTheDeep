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

    // Flag to ensure we have initialized (claimed) the LED driver.
    private boolean isInitialized = false;

    public LEDs(HardwareMap hw, Sensors sensors, Intake intake, Depositor depositor, Telemetry telemetry) {
        this.ledDriver = hw.get(RevBlinkinLedDriver.class, Constants.HardwareMapping.leds);
        this.mSensors = sensors;
        this.mIntake = intake;
        this.mDepositor = depositor;
        this.mTelemetry = telemetry;
    }

    /**
     * Call this method during your OpMode init() phase to “claim” the LED driver.
     * By sending a PWM signal that matches the LED’s boot‐up default pattern,
     * you ensure that subsequent PWM commands (even if different) will update the LEDs.
     */
    public void initializeLEDs() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
        isInitialized = true;
    }

    @Override
    public void periodic() {
        // Only update LED patterns after initialization (claiming) is complete.
        if (!isInitialized) {
            return;
        }

        GamePieceColor intakeColor = mIntake.pieceColorSeen();
        GamePieceColor depositorColor = mDepositor.pieceColorSeen();

        if (Constants.DebugModes.DEBUG_TELEMETRY) {
            mTelemetry.addData("LEDs: Intake Blue", intakeColor == GamePieceColor.BLUE);
            mTelemetry.addData("LEDs: Intake Red", intakeColor == GamePieceColor.RED);
            mTelemetry.addData("LEDs: Intake Yellow", intakeColor == GamePieceColor.YELLOW);
            mTelemetry.addData("LEDs: Depositor Blue", depositorColor == GamePieceColor.BLUE);
            mTelemetry.addData("LEDs: Depositor Red", depositorColor == GamePieceColor.RED);
            mTelemetry.addData("LEDs: Depositor Yellow", depositorColor == GamePieceColor.YELLOW);
        }

        if (intakeColor == GamePieceColor.RED) setRed();
        else if (intakeColor == GamePieceColor.BLUE) setBlue();
        else if (intakeColor == GamePieceColor.YELLOW) setYellow();
        else if (depositorColor == GamePieceColor.RED) setRedHeartbeat();
        else if (depositorColor == GamePieceColor.BLUE) setBlueHeartbeat();
        else if (depositorColor == GamePieceColor.YELLOW) setYellowHeartbeat();
        else setSinelonOcean();
    }

    // LED pattern methods:

    public void setBpmOcean() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
    }

    public void setWavesOcean() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
    }

    public void setTwinklesOcean() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE);
    }

    public void setSinelonOcean() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
    }

    public void setRainbowOcean() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
    }

    public void setSilver() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
    }

    public void setYellow() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void setRed() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void setBlue() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void setBlack() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setYellowHeartbeat() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
    }

    public void setRedHeartbeat() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
    }

    public void setBlueHeartbeat() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
    }

    public void setYellowChase() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
    }

    public void setRedChase() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
    }

    public void setBlueChase() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.ledDriver.setPattern(pattern);
    }
}
