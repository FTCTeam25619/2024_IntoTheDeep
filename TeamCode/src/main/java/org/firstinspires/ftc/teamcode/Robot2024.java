package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.commands.DriveRobot;
import org.firstinspires.ftc.teamcode.commands.MoveLiftDown;
import org.firstinspires.ftc.teamcode.commands.MoveLiftUp;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Constants.OpModes.OpModeSelection;
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
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        // Subsystems
        sensors = new Sensors(hardwareMap, gyroOrientation, Robot2024.telemetry);
        drivetrain = new Drivetrain(hardwareMap, sensors, robotState, Robot2024.telemetry);
        lift = new Lift(hardwareMap, sensors, Robot2024.telemetry);

        // Controllers
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);
    }

    public void initOpMode() {
        switch (selectedOpMode) {
            case DRIVE_STICKS_TELEOP:
                CommandScheduler.getInstance().schedule(
                        new DriveRobot(drivetrain, controller1, robotState, Robot2024.telemetry));
        }
        setupGamepadButtonMappings();
    }

    public void setupGamepadButtonMappings() {
        GamepadButton c1LeftBumper = new GamepadButton(controller1, GamepadKeys.Button.LEFT_BUMPER);
        c1LeftBumper.whenPressed(new InstantCommand(() -> robotState.setSlowDriveMode(true)));
        c1LeftBumper.whenReleased(new InstantCommand(() -> robotState.setSlowDriveMode(false)));

        GamepadButton c1DPadDown = new GamepadButton(controller1, GamepadKeys.Button.DPAD_DOWN);
        c1DPadDown.whenPressed(new InstantCommand(() -> robotState.toggleFieldCentric()));

        GamepadButton c2DPadUp = new GamepadButton(controller2, GamepadKeys.Button.DPAD_UP);
        GamepadButton c2DPadDown = new GamepadButton(controller2, GamepadKeys.Button.DPAD_DOWN);

        c2DPadUp.whileHeld(new MoveLiftUp(lift));
        c2DPadDown.whileHeld(new MoveLiftDown(lift));
    }
}