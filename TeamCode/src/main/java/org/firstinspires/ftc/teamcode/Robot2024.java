package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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

import org.firstinspires.ftc.teamcode.commands.AwaitGamePiece;
import org.firstinspires.ftc.teamcode.commands.DriveRobot;
import org.firstinspires.ftc.teamcode.commands.IntakePiece;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToHeight;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;


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
    }

    public void initOpMode() {
        switch (selectedOpMode) {
            case DRIVE_STICKS_TELEOP:
                drivetrain.setDefaultCommand(new DriveRobot(drivetrain, controller1, robotState, Robot2024.telemetry));
                intake.setDefaultCommand(new IntakePiece(intake, controller2, Robot2024.telemetry));
        }
        setupGamepadButtonMappings();
    }

    public void resetServos() {
        intake.pivotToPosition(Constants.Intake.PivotSetPosition.UP);
        intake.slideToPosition(Constants.Intake.SlideSetPosition.IN);
        depositor.gripToPosition(Constants.Depositor.GripSetPosition.OPEN);
        depositor.wristToPosition(Constants.Depositor.WristSetPosition.INTAKE);
        depositor.armToPosition(Constants.Depositor.ArmSetPosition.HOME);
    }

    public void neutralServos() {
        intake.pivotToPosition(Constants.Intake.PivotSetPosition.UP);
        intake.slideToPosition(Constants.Intake.SlideSetPosition.NEUTRAL);
        depositor.gripToPosition(Constants.Depositor.GripSetPosition.OPEN);
        depositor.wristToPosition(Constants.Depositor.WristSetPosition.INTAKE);
        depositor.armToPosition(Constants.Depositor.ArmSetPosition.NEUTRAL);
    }

    public void setupGamepadButtonMappings() {
        // Setup controller 1 buttons
        GamepadButton c1DPadUp = new GamepadButton(controller1, GamepadKeys.Button.DPAD_UP);
        GamepadButton c1DPadDown = new GamepadButton(controller1, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c1A = new GamepadButton(controller1, GamepadKeys.Button.A);
        GamepadButton c1B = new GamepadButton(controller1, GamepadKeys.Button.B);
        GamepadButton c1X = new GamepadButton(controller1, GamepadKeys.Button.X);
        GamepadButton c1Y = new GamepadButton(controller1, GamepadKeys.Button.Y);
        GamepadButton c1LeftBumper = new GamepadButton(controller1, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c1RightBumper = new GamepadButton(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton c1Back = new GamepadButton(controller1, GamepadKeys.Button.BACK);
        GamepadButton c1Start = new GamepadButton(controller1, GamepadKeys.Button.START);
        // Setup controller 2 buttons
        GamepadButton c2DPadUp = new GamepadButton(controller2, GamepadKeys.Button.DPAD_UP);
        GamepadButton c2DPadDown = new GamepadButton(controller2, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c2A = new GamepadButton(controller2, GamepadKeys.Button.A);
        GamepadButton c2B = new GamepadButton(controller2, GamepadKeys.Button.B);
        GamepadButton c2X = new GamepadButton(controller2, GamepadKeys.Button.X);
        GamepadButton c2Y = new GamepadButton(controller2, GamepadKeys.Button.Y);
        GamepadButton c2LeftBumper = new GamepadButton(controller2, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c2RightBumper = new GamepadButton(controller2, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton c2Back = new GamepadButton(controller2, GamepadKeys.Button.BACK);
        GamepadButton c2Start = new GamepadButton(controller2, GamepadKeys.Button.START);

        // Map controller 1 buttons
        c1DPadDown.whenPressed(new InstantCommand(() -> robotState.toggleFieldCentric()));

        c1A.whileHeld(new InstantCommand(() -> depositor.awaitPiece()));

        // Neutral mode (retract slide/intake, arm to neutral)
        c1X.whenPressed(this::neutralServos);
        // Reset to start position
        c1Y.whenPressed(this::resetServos);

        c1LeftBumper.whenPressed(new InstantCommand(() -> robotState.setSlowDriveMode(true)));
        c1LeftBumper.whenReleased(new InstantCommand(() -> robotState.setSlowDriveMode(false)));

        // Extend intake
        c1RightBumper.whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> intake.slideToPosition(Constants.Intake.SlideSetPosition.OUT)),
                        new WaitCommand(10),
                        new InstantCommand(() -> intake.pivotToPosition(Constants.Intake.PivotSetPosition.DOWN)),
                        new InstantCommand(() -> depositor.gripToPosition(Constants.Depositor.GripSetPosition.OPEN)),
                        new InstantCommand(() -> depositor.armToPosition(Constants.Depositor.ArmSetPosition.HOME)),
                        new InstantCommand(() -> depositor.wristToPosition(Constants.Depositor.WristSetPosition.INTAKE))
                )
        );

        // Pivot position tuning
//        c1Back.whenPressed(new InstantCommand(() -> intake.pivotLeftToTestPosition()));
//        c1Start.whenPressed(new InstantCommand(() -> intake.pivotRightToTestPosition()));
        // Slide position tuning
//        c1Back.whenPressed(new InstantCommand(() -> intake.slideLeftToTestPosition()));
//        c1Start.whenPressed(new InstantCommand(() -> intake.slideRightToTestPosition()));
        // Arm position tuning
//        c1Back.whenPressed(new InstantCommand(() -> depositor.armLeftToTestPosition()));
//        c1Start.whenPressed(new InstantCommand(() -> depositor.armRightToTestPosition()));
        // Grip & Wrist position tuning
//        c1Back.whenPressed(new InstantCommand(() -> depositor.gripToTestPosition()));
//        c1Start.whenPressed(new InstantCommand(() -> depositor.wristToTestPosition()));

        // Map controller 2 buttons
        c2DPadUp.whileHeld(new InstantCommand(() -> lift.setMotorPower(ConfigConstants.ManualMovement.liftUpMotorPower)));
        c2DPadUp.whenReleased(new InstantCommand(lift::stopMotors));
        c2DPadDown.whileHeld(new InstantCommand(() -> lift.setMotorPower(ConfigConstants.ManualMovement.liftDownMotorPower)));
        c2DPadDown.whenReleased(new InstantCommand(lift::stopMotors));


//        c2B.whileHeld(new InstantCommand(() -> intake.intakePiece(), intake));

        // Neutral position
        c2X.whenPressed(new InstantCommand(this::neutralServos));
        // Re-home/reset to start positions
        c2Y.whenPressed(new InstantCommand(this::resetServos));

        // TODO:  Temporary settings for testing Lift PID
        c2A.whenPressed(new MoveLiftToHeight(lift, ConfigConstants.Lift.LIFT_DOWN_POS));
//        c2B.whenPressed(new MoveLiftToHeight(lift, ConfigConstants.Lift.LIFT_LOW_BASKET));
        c2B.whenPressed(new MoveLiftToHeight(lift, ConfigConstants.Lift.LIFT_HI_BASKET));

        // After piece intake, handoff to depositor
        c2LeftBumper.whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> intake.pivotToPosition(Constants.Intake.PivotSetPosition.UP)),
                        new WaitCommand(850),
                        new InstantCommand(() -> intake.slideToPosition(Constants.Intake.SlideSetPosition.IN)),
                        new WaitCommand(50),
                        new AwaitGamePiece(depositor),
                        new InstantCommand(() -> intake.slideToPosition(Constants.Intake.SlideSetPosition.NEUTRAL)),
                        new InstantCommand(() -> depositor.armToPosition(Constants.Depositor.ArmSetPosition.NEUTRAL))
                )
        );

        // Press & hold to move to scoring position, release to score
        c2RightBumper.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intake.slideToPosition(Constants.Intake.SlideSetPosition.NEUTRAL);
                    depositor.armToPosition(Constants.Depositor.ArmSetPosition.NEUTRAL);
                }),
                new WaitCommand(ConfigConstants.ScoringTiming.preScoreWaitForLiftMS),
                new ParallelCommandGroup(
                        new MoveLiftToHeight(lift, ConfigConstants.Lift.LIFT_HI_BASKET),
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
                )
        ));
        c2RightBumper.whenReleased(
                new ParallelCommandGroup(
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
                )
        );

//        c2LeftBumper.whileHeld(new InstantCommand(() -> intake.slideLeftToTestPosition()));
//        c2LeftBumper.whileHeld(new InstantCommand(() -> depositor.armToPosition(Constants.Depositor.ArmSetPosition.HOME)));

//        c2RightBumper.whileHeld(new InstantCommand(() -> intake.slideRightToTestPosition()));
//        c2RightBumper.whileHeld(new InstantCommand(() -> depositor.armToPosition(Constants.Depositor.ArmSetPosition.SCORING)));

//        c2Back.whileHeld(new InstantCommand(() -> depositor.armRightToTestPosition()));
//        c2Back.whileHeld(new InstantCommand(() -> depositor.armToPosition(Constants.Depositor.ArmSetPosition.NEUTRAL)));
//
//        c2Start.whileHeld(new InstantCommand(() -> depositor.armToPosition(Constants.Depositor.ArmSetPosition.HOME)));

        c2Back.whileHeld(new InstantCommand(() -> {
            climb.enablePIDHold(false);
            climb.setMotorPower(ConfigConstants.ManualMovement.climbUpMotorPower);
        }));
        c2Back.whenReleased(new SequentialCommandGroup(
                new InstantCommand(() -> climb.enablePIDHold(true)),
                new WaitCommand(ConfigConstants.Climb.holdTimeoutMS),
                new InstantCommand(() -> climb.stopMotors())
        ));

        c2Start.whileHeld(new InstantCommand(() -> {
            climb.enablePIDHold(false);
            climb.setMotorPower(ConfigConstants.ManualMovement.climbDownMotorPower);
        }));
        c2Start.whenReleased(new SequentialCommandGroup(
                new InstantCommand(() -> climb.enablePIDHold(true)),
                new WaitCommand(ConfigConstants.Climb.holdTimeoutMS),
                new InstantCommand(() -> climb.stopMotors())
        ));
    }
}
