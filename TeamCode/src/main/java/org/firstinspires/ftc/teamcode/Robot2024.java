package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.commands.DriveRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;


/* To connect to the Control Hub device via Wi-Fi:
   - Connect Wi-Fi to the FTC-25619 network
   - In command line, run: adb connect 192.168.43.1:5555
   - To check that it is connected, run: adb devices -l
   - Check that the REV Robotics Control Hub shows up above
   - in Android Studio with a green dot (connected)
 */
 
public class Robot2024 extends Robot {
    private final GamepadEx controller1;
    private final Sensors sensors;
    private final Drivetrain drivetrain;
    private final RevHubOrientationOnRobot gyroOrientation;
    public static Telemetry telemetry;

    private final OpModeSelection selectedOpMode;

    public Robot2024(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry, OpModeSelection opModeSelection) {
        // OpMode selection
        selectedOpMode = opModeSelection;

        // Telemetry
        Robot2024.telemetry = telemetry;

        // Gyro Orientation on Robot
        gyroOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        // Subsystems
        sensors = new Sensors(hardwareMap, gyroOrientation);
        drivetrain = new Drivetrain(hardwareMap, sensors, Robot2024.telemetry);

        // Controllers
        controller1 = new GamepadEx(gamepad1);
    }

    public void initOpMode() {
        switch (selectedOpMode) {
            case DRIVE_STICKS_TELEOP:
                CommandScheduler.getInstance().schedule(new DriveRobot(drivetrain, controller1));
        }
    }
}