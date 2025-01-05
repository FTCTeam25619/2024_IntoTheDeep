package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This class maintains needed robot state.
 */
public class RobotState {
    public boolean slowDriveMode;
    public DriveMode robotDriveMode;

    private final Telemetry mTelemetry;

    public RobotState(Telemetry robotTelemetry) {
        this.mTelemetry = robotTelemetry;

        this.slowDriveMode = false;
        this.robotDriveMode = DriveMode.FIELD_CENTRIC_FTCLIB;

        mTelemetry.addData("RobotState: SLOW MODE", this.slowDriveMode);
        mTelemetry.addData("RobotState: FIELD CENTRIC", this.robotDriveMode.fieldCentric);
        mTelemetry.addData("RobotState: FTCLIB DRIVE CODE", this.robotDriveMode.ftcLibDriveControl);
    }

    public void setSlowDriveMode(boolean driveSlow) {
        this.slowDriveMode = driveSlow;
    }

    public void toggleFieldCentric() {
        if (this.robotDriveMode.ftcLibDriveControl) {
            this.robotDriveMode = this.robotDriveMode.fieldCentric ?
                    DriveMode.ROBOT_CENTRIC_FTCLIB :
                    DriveMode.FIELD_CENTRIC_FTCLIB;
        } else {
            this.robotDriveMode = this.robotDriveMode.fieldCentric ?
                    DriveMode.ROBOT_CENTRIC_CUSTOM :
                    DriveMode.FIELD_CENTRIC_CUSTOM;
        }
    }

    public enum DriveMode {
        ROBOT_CENTRIC_CUSTOM(false, false, false),
        FIELD_CENTRIC_CUSTOM(true, true, false),
        ROBOT_CENTRIC_FTCLIB(false, false, true),
        FIELD_CENTRIC_FTCLIB(true, true, true);

        public final boolean fieldCentric;
        public final boolean fieldCentricPointToTurn;
        public final boolean ftcLibDriveControl;

        private DriveMode(boolean isFieldCentric, boolean usePointToTurn, boolean useFTCLib) {
            this.fieldCentric = isFieldCentric;
            this.fieldCentricPointToTurn = isFieldCentric && usePointToTurn;
            this.ftcLibDriveControl = useFTCLib;
        }
    }
}
