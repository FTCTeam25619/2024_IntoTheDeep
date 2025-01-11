package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.commands.AutoIntakePiece;
import org.firstinspires.ftc.teamcode.commands.AwaitGamePiece;
import org.firstinspires.ftc.teamcode.commands.DriveRobot;
import org.firstinspires.ftc.teamcode.commands.HoldClimb;
import org.firstinspires.ftc.teamcode.lib.triggers.LeftStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.RightStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerAxis;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerButton;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToHeight;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LEDs;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Sweep;

import java.util.function.DoubleSupplier;


/* To connect to the Control Hub device via Wi-Fi:
   - Connect Wi-Fi to the FTC-25619 network
   - In command line, run: adb connect 192.168.43.1:5555
   - To check that it is connected, run: adb devices -l
   - Check that the REV Robotics Control Hub shows up above
   - in Android Studio with a green dot (connected)
 */

public class Robot2024 extends Robot {
    private final RobotState robotState;
    private final GamepadEx controller1;
    private final GamepadEx controller2;
    private final Sensors sensors;
    private final Drivetrain drivetrain;
    private final Lift lift;
    private final Intake intake;
    private final Depositor depositor;
    private final Climb climb;
    private final Sweep sweep;
    private final LEDs leds;
    private final RevHubOrientationOnRobot gyroOrientation;
    public static Telemetry telemetry;

    private final OpModeSelection selectedOpMode;

    public Robot2024(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, OpModeSelection opModeSelection) {
        // OpMode selection
        selectedOpMode = opModeSelection;

        // Telemetry
        Robot2024.telemetry = telemetry;

        // Robot state
        robotState = new RobotState(Robot2024.telemetry);

        // Gyro Orientation on Robot
        gyroOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        // Controllers
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        // Subsystems
        sensors = new Sensors(hardwareMap, gyroOrientation, Robot2024.telemetry);
        drivetrain = new Drivetrain(hardwareMap, sensors, robotState, Robot2024.telemetry);
        lift = new Lift(hardwareMap, sensors, Robot2024.telemetry);
        intake = new Intake(hardwareMap, Robot2024.telemetry);
        depositor = new Depositor(hardwareMap, Robot2024.telemetry);
        climb = new Climb(hardwareMap, sensors, Robot2024.telemetry);
        sweep = new Sweep(hardwareMap, Robot2024.telemetry);
        leds = new LEDs(hardwareMap, sensors, intake, depositor, Robot2024.telemetry);
    }

    public void initOpMode() {
        switch (selectedOpMode) {
            case DRIVE_STICKS_TELEOP:
                // Servos to Home positions
                resetServos();
                // Left and Right Sticks
                drivetrain.setDefaultCommand(new DriveRobot(drivetrain, controller1, robotState, Robot2024.telemetry));
        }
        setupGamepadButtonMappings();
    }

    public void resetServos() {
        intake.stopIntake();
        intake.pivotToPosition(Constants.Intake.PivotSetPosition.UP);
        intake.slideToPosition(Constants.Intake.SlideSetPosition.IN);
        depositor.gripToPosition(Constants.Depositor.GripSetPosition.OPEN);
        depositor.wristToPosition(Constants.Depositor.WristSetPosition.INTAKE);
        depositor.armToPosition(Constants.Depositor.ArmSetPosition.HOME);
        sweep.sweepToPosition(Constants.Depositor.SweepSetPosition.HOME);
    }

    public void neutralServos() {
        intake.stopIntake();
        intake.pivotToPosition(Constants.Intake.PivotSetPosition.UP);
        intake.slideToPosition(Constants.Intake.SlideSetPosition.NEUTRAL);
        depositor.gripToPosition(Constants.Depositor.GripSetPosition.OPEN);
        depositor.wristToPosition(Constants.Depositor.WristSetPosition.INTAKE);
        depositor.armToPosition(Constants.Depositor.ArmSetPosition.NEUTRAL);
        sweep.sweepToPosition(Constants.Depositor.SweepSetPosition.HOME);
    }

    public void setupGamepadButtonMappings() {
        setupController1ButtonMappings();
        setupController2ButtonMappings();

//        setupTestingButtonMappings();
    }

    private void setupController1ButtonMappings() {
        // Setup controller 1 buttons
        GamepadButton c1DPadUp = new GamepadButton(controller1, GamepadKeys.Button.DPAD_UP);
        GamepadButton c1DPadDown = new GamepadButton(controller1, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c1DPadLeft = new GamepadButton(controller1, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton c1DPadRight = new GamepadButton(controller1, GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton c1A = new GamepadButton(controller1, GamepadKeys.Button.A);
        GamepadButton c1B = new GamepadButton(controller1, GamepadKeys.Button.B);
        GamepadButton c1X = new GamepadButton(controller1, GamepadKeys.Button.X);
        GamepadButton c1Y = new GamepadButton(controller1, GamepadKeys.Button.Y);
        GamepadButton c1LeftBumper = new GamepadButton(controller1, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c1RightBumper = new GamepadButton(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        TriggerAxis c1LeftTrigger = new TriggerAxis(controller1, GamepadKeys.Trigger.LEFT_TRIGGER, ConfigConstants.ManualMovement.slideManualThreshold);
        TriggerAxis c1RightTrigger = new TriggerAxis(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER, ConfigConstants.ManualMovement.slideManualThreshold);
        GamepadButton c1LeftStickPress = new GamepadButton(controller1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        GamepadButton c1RightStickPress = new GamepadButton(controller1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        GamepadButton c1Back = new GamepadButton(controller1, GamepadKeys.Button.BACK);
        GamepadButton c1Start = new GamepadButton(controller1, GamepadKeys.Button.START);

        // Map controller 1 buttons to commands
        // Triggers are setup via default command as slide control
        // Joysticks are setup via default command as drive control

        // Press and hold L bumper or either stick to execute slow mode
        c1LeftBumper.whenPressed(slowDriveModeOn());
        c1LeftStickPress.whenPressed(slowDriveModeOn());
        c1RightStickPress.whenPressed((slowDriveModeOn()));
        // Slow mode will turn off when released
        c1LeftBumper.whenReleased(slowDriveModeOff());
        c1LeftStickPress.whenReleased(slowDriveModeOff());
        c1RightStickPress.whenReleased(slowDriveModeOff());

        // Right bumper will release piece and retract scoring mechanism
        c1RightBumper.whenPressed(scoreBasketAndReturnHome());

        c1LeftTrigger.whileActiveContinuous(manualRetractSlide(c1LeftTrigger));
        c1RightTrigger.whileActiveContinuous(manualExtendSlide(c1RightTrigger));

        // Dissection mode: full extent for inspection
        c1Back.whenPressed(dissection());
        // Reset the Gyro for field-oriented drive
        c1Start.whenPressed(resetGyro());
    }

    private void setupController2ButtonMappings() {
        // Setup controller 2 buttons
        LeftStickY c2LeftStickYUp = new LeftStickY(controller2, 0.6);
        LeftStickY c2LeftStickYDown = new LeftStickY(controller2, -0.6);
        RightStickY c2RightStickYUp = new RightStickY(controller2, 0.6);
        RightStickY c2RightStickYDown = new RightStickY(controller2, -0.6);
        GamepadButton c2DPadUp = new GamepadButton(controller2, GamepadKeys.Button.DPAD_UP);
        GamepadButton c2DPadDown = new GamepadButton(controller2, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c2DPadLeft = new GamepadButton(controller2, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton c2DPadRight = new GamepadButton(controller2, GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton c2A = new GamepadButton(controller2, GamepadKeys.Button.A);
        GamepadButton c2B = new GamepadButton(controller2, GamepadKeys.Button.B);
        GamepadButton c2X = new GamepadButton(controller2, GamepadKeys.Button.X);
        GamepadButton c2Y = new GamepadButton(controller2, GamepadKeys.Button.Y);
        GamepadButton c2LeftBumper = new GamepadButton(controller2, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c2RightBumper = new GamepadButton(controller2, GamepadKeys.Button.RIGHT_BUMPER);
        TriggerButton c2LeftTrigger = new TriggerButton(controller2, GamepadKeys.Trigger.LEFT_TRIGGER, 0.6);
        TriggerButton c2RightTrigger = new TriggerButton(controller2, GamepadKeys.Trigger.RIGHT_TRIGGER, 0.6);
        GamepadButton c2LeftStickPress = new GamepadButton(controller2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        GamepadButton c2RightStickPress = new GamepadButton(controller2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        GamepadButton c2Back = new GamepadButton(controller2, GamepadKeys.Button.BACK);
        GamepadButton c2Start = new GamepadButton(controller2, GamepadKeys.Button.START);

        // Map controller 2 buttons to commands
        // Left stick is mapped to Manual Lift movement up and down
        c2LeftStickYUp.whileActiveContinuous(moveLiftUp());
        c2LeftStickYUp.whenInactive(stopLift());
        c2LeftStickYDown.whileActiveContinuous(moveLiftDown());
        c2LeftStickYDown.whenInactive(stopLift());
        // Right stick is mapped to Manual Climb hooks movement up and down
        c2RightStickYUp.whileActiveContinuous(moveClimbHooksUp());
        c2RightStickYUp.whenInactive(holdClimb());
        c2RightStickYDown.whileActiveContinuous(moveClimbHooksDown());
        c2RightStickYDown.whenInactive(holdClimb());
        // Right and left triggers are mapped to Manual Intake wheels movement in (intake) and out (eject) respectively

        // Press and hold DPad Up to sweep in front of the robot. Sweeper bar will return when released.
        c2DPadUp.whenPressed(sweepOut());
        c2DPadUp.whenReleased(sweepIn());

        // A B X Y move the lift and depositor to scoring positions
        // B and Y do low and high basket positions respectively
        // A and X will do low and high clip bars respectively, when that sequence is created
        c2B.whenPressed(moveToScoringPosition(Constants.ScoringPosition.LOW_BASKET));
        c2Y.whenPressed(moveToScoringPosition(Constants.ScoringPosition.HIGH_BASKET));

        c2LeftBumper.whenPressed(handoffPiece());
        c2RightBumper.whenPressed(extendIntake());

        c2LeftTrigger.whileActiveContinuous(manualOuttake());
        c2LeftTrigger.whenInactive(stopIntakeWheels());

        c2RightTrigger.whileActiveContinuous(manualIntake());
        c2RightTrigger.whenInactive(stopIntakeWheels());

        c2Back.whenPressed(neutralPosition());
        c2Start.whenPressed(resetPosition());
    }

    private void setupTestingButtonMappings() {
        // Test positions for tuning
        GamepadButton c1A = new GamepadButton(controller1, GamepadKeys.Button.A);
        GamepadButton c1B = new GamepadButton(controller1, GamepadKeys.Button.B);
        // Pivot position tuning
//        c1A.whenPressed(new InstantCommand(() -> intake.pivotLeftToTestPosition()));
//        c1B.whenPressed(new InstantCommand(() -> intake.pivotRightToTestPosition()));
        // Slide position tuning
//        c1A.whenPressed(new InstantCommand(() -> intake.slideLeftToTestPosition()));
//        c1B.whenPressed(new InstantCommand(() -> intake.slideRightToTestPosition()));
        // Arm position tuning
//        c1A.whenPressed(new InstantCommand(() -> depositor.armLeftToTestPosition()));
//        c1B.whenPressed(new InstantCommand(() -> depositor.armRightToTestPosition()));
        // Grip & Wrist position tuning
//        c1A.whenPressed(new InstantCommand(() -> depositor.gripToTestPosition()));
//        c1B.whenPressed(new InstantCommand(() -> depositor.wristToTestPosition()));
        // Sweep position tuning
//        c1A.whenPressed(new InstantCommand(() -> sweep.sweepToTestPosition()));
    }

    Command slowDriveModeOn() {
        return new InstantCommand(() -> robotState.setSlowDriveMode(true));
    }

    Command slowDriveModeOff() {
        return new InstantCommand(() -> robotState.setSlowDriveMode(false));
    }

    Command neutralPosition() {
        return new InstantCommand(this::neutralServos, intake, depositor, sweep);
    }

    Command resetPosition() {
        return new InstantCommand(this::resetServos, intake, depositor, sweep);
    }

    Command dissection() {
        return new SequentialCommandGroup(
            extendIntakeToSlidePosition(Constants.Intake.SlideSetPosition.OUT_FAR),
            new WaitCommand(50),
            moveLiftAndDepositorToScoringPosition(Constants.ScoringPosition.HIGH_BASKET)
        );
    }

    Command extendIntakeToSlidePosition(Constants.Intake.SlideSetPosition slidePosition) {
        return new SequentialCommandGroup(
                slowDriveModeOn(),
                new InstantCommand(() -> intake.slideToPosition(slidePosition)),
                new InstantCommand(() -> intake.pivotToPosition(Constants.Intake.PivotSetPosition.DOWN)),
                new InstantCommand(() -> depositor.gripToPosition(Constants.Depositor.GripSetPosition.OPEN)),
                new InstantCommand(() -> depositor.armToPosition(Constants.Depositor.ArmSetPosition.HOME)),
                new InstantCommand(() -> depositor.wristToPosition(Constants.Depositor.WristSetPosition.INTAKE))
        );
    }
    Command extendIntake() {
        return new ParallelCommandGroup(
                extendIntakeToSlidePosition(Constants.Intake.SlideSetPosition.OUT_NEAR),
                new AutoIntakePiece(intake, Robot2024.telemetry)
        );
    }

    Command handoffPiece() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.pivotToPosition(Constants.Intake.PivotSetPosition.UP)),
                new WaitCommand(ConfigConstants.IntakeTiming.handoffWaitBeforeSlideMove),
                new InstantCommand(() -> intake.slideToPosition(Constants.Intake.SlideSetPosition.IN)),
                new WaitCommand(ConfigConstants.IntakeTiming.handoffWaitForMateMS),
                manualIntake(),
                new AwaitGamePiece(depositor),
                stopIntakeWheels(),
                slowDriveModeOff()
        );
    }

    Command sweepOut() {
         return new InstantCommand(() -> sweep.sweepToPosition(Constants.Depositor.SweepSetPosition.SWEEP));
    }

    Command sweepIn() {
        return new InstantCommand(() -> sweep.sweepToPosition(Constants.Depositor.SweepSetPosition.HOME));
    }

    Command moveLiftAndDepositorToScoringPosition(Constants.ScoringPosition scoringPosition) {
        return new ParallelCommandGroup(
                new MoveLiftToHeight(lift, scoringPosition.liftPosition),
                new SequentialCommandGroup(
                        new WaitCommand(ConfigConstants.ScoringTiming.preScoreWaitForArmMS),
                        new InstantCommand(() -> {
                            depositor.gripToPosition(Constants.Depositor.GripSetPosition.CLOSED);
                            depositor.armToPosition(Constants.Depositor.ArmSetPosition.SCORING);
                        }),
                        new WaitCommand(ConfigConstants.ScoringTiming.preScoreWaitForWristMS),
                        new InstantCommand(() -> {
                            depositor.wristToPosition(Constants.Depositor.WristSetPosition.SCORING);
                        })
                )
        );
    }

    Command moveToScoringPosition(Constants.ScoringPosition scoringPosition) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intake.slideToPosition(Constants.Intake.SlideSetPosition.NEUTRAL);
                    depositor.armToPosition(Constants.Depositor.ArmSetPosition.NEUTRAL);
                }),
                new WaitCommand(ConfigConstants.ScoringTiming.preScoreWaitForLiftMS),
                moveLiftAndDepositorToScoringPosition(scoringPosition)
        );
    }

    Command scoreBasketAndReturnHome() {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(() -> depositor.gripToPosition(Constants.Depositor.GripSetPosition.OPEN)),
                        new WaitCommand(ConfigConstants.ScoringTiming.postScoreWaitForArmMS),
                        new InstantCommand(() -> depositor.armToPosition(Constants.Depositor.ArmSetPosition.NEUTRAL)),
                        new WaitCommand(ConfigConstants.ScoringTiming.postScoreWaitForWristMS),
                        new InstantCommand(() -> depositor.wristToPosition(Constants.Depositor.WristSetPosition.INTAKE)),
                        new WaitCommand(ConfigConstants.ScoringTiming.postScoreWaitForHomeMS),
                        new InstantCommand(() -> {
                            intake.pivotToPosition(Constants.Intake.PivotSetPosition.UP);
                            intake.slideToPosition(Constants.Intake.SlideSetPosition.IN);
                            depositor.armToPosition(Constants.Depositor.ArmSetPosition.HOME);
                        })
                ),
                new SequentialCommandGroup(
                        new WaitCommand(ConfigConstants.ScoringTiming.postScoreWaitForLiftMS),
                        new MoveLiftToHeight(lift, ConfigConstants.Lift.LIFT_DOWN_POS)
                )
        );
    }

    Command moveClimbHooksUp() {
        return new InstantCommand(() -> {
            climb.enablePIDHold(false);
            climb.setMotorPower(ConfigConstants.ManualMovement.climbUpMotorPower);
        }, climb);
    }

    Command moveClimbHooksDown() {
        return new InstantCommand(() -> {
            climb.enablePIDHold(false);
            climb.setMotorPower(ConfigConstants.ManualMovement.climbDownMotorPower);
        }, climb);
    }

    Command holdClimb() {
        return new HoldClimb(climb, ConfigConstants.Climb.holdTimeoutMS, Robot2024.telemetry);
    }

    Command moveLiftUp() {
        return new InstantCommand(() -> lift.setMotorPower(ConfigConstants.ManualMovement.liftUpMotorPower));
    }

    Command moveLiftDown() {
        return new InstantCommand(() -> lift.setMotorPower(ConfigConstants.ManualMovement.liftDownMotorPower));
    }

    Command stopLift() {
        return new InstantCommand(lift::stopMotors);
    }

    Command manualIntake() {
        return new InstantCommand(intake::intakePiece);
    }

    Command manualOuttake() {
        return new InstantCommand(intake::outtakePiece);
    }

    Command stopIntakeWheels() {
        return new InstantCommand(intake::stopIntake);
    }

    Command manualExtendSlide(DoubleSupplier axisSupplier) {
        return new InstantCommand(() -> {
            intake.moveSlideManual(axisSupplier, ConfigConstants.ManualMovement.slideManualIncrementExtend);
        });
    }

    Command manualRetractSlide(DoubleSupplier axisSupplier) {
        return new InstantCommand(() -> {
            intake.moveSlideManual(axisSupplier, ConfigConstants.ManualMovement.slideManualIncrementRetract);
        });
    }

    Command resetGyro() {
        return new InstantCommand(sensors::resetGyro);
    }
}
