package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.Robot2024;

@Autonomous
public class AutoTest extends OpMode {
        private Robot2024 robot;

        @Override
        public void init() {
            // Create FTC Dashboard if setup for shop debugging
            if (Constants.DebugModes.ENABLE_FTC_DASHBOARD) {
                FtcDashboard dashboard = FtcDashboard.getInstance();
                telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
            }

            robot = new Robot2024(hardwareMap, gamepad1, gamepad2, telemetry, OpModeSelection.TEST_AUTO);

            telemetry.addData("Robot Status", "Initializing AutoTest");
        }

        @Override
        public void init_loop() {
            // Fancy LED loop that tells us which OpMode is active and wows the audience
            // before we hit the play button at the match start
        }

        @Override
        public void start() {
            telemetry.addData("Robot Status", "Running AutoTest");
            robot.initOpMode();
        }

        @Override
        public void loop() {
            robot.run();
        }

        @Override
        public void stop() {
            telemetry.addData("Robot Status", "Stopped AutoTest");
            robot.reset();
        }

}
