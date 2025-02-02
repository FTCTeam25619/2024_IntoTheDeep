
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.Robot2024;

@TeleOp
public class TeleOpDriveSticks_BLUE extends OpMode {
    private Robot2024 robot;

    @Override
    public void init() {
        // Create FTC Dashboard if setup for shop debugging
        if (Constants.DebugModes.ENABLE_FTC_DASHBOARD) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        robot = new Robot2024(hardwareMap, gamepad1, gamepad2, telemetry, OpModeSelection.DRIVE_STICKS_TELEP_BLUE);

        telemetry.addData("Robot Status", "Initializing TeleOpDriveSticks");
    }

    @Override
    public void init_loop() {
        // Fancy LED loop that tells us which OpMode is active and wows the audience
        // before we hit the play button at the match start
    }

    @Override
    public void start() {
        telemetry.addData("Robot Status", "Running TeleOpDriveSticks");
        robot.initOpMode();
    }

    @Override
    public void loop() {
        robot.run();
    }

    @Override
    public void stop() {
        telemetry.addData("Robot Status", "Stopped TeleOpDriveSticks");
        robot.reset();
    }
}
