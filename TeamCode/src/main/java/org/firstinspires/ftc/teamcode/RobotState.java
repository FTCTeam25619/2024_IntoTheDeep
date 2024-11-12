package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This class maintains needed robot state.
 */
public class RobotState {
    public boolean slowDriveMode = false;
    public DriveMode robotDriveMode = DriveMode.FIELD_CENTRIC_CUSTOM;

    private Telemetry mTelemetry;

    public RobotState(Telemetry robotTelemetry) {
        this.mTelemetry = robotTelemetry;

        this.slowDriveMode = false;
        this.robotDriveMode = DriveMode.FIELD_CENTRIC_CUSTOM;

        mTelemetry.addData("RobotState: SLOW MODE", this.slowDriveMode);
    }

    public void setSlowDriveMode(boolean driveSlow) {
        this.slowDriveMode = driveSlow;

        mTelemetry.addData("RobotState: SLOW MODE", this.slowDriveMode);
        mTelemetry.addData("RobotState: FIELD CENTRIC", this.robotDriveMode.fieldCentric);
        mTelemetry.addData("RobotState: FTCLIB DRIVE CODE", this.robotDriveMode.ftcLibDriveControl);
    }

    public void toggleFieldCentric() {
        switch(this.robotDriveMode) {
            case FIELD_CENTRIC_CUSTOM:
            case ROBOT_CENTRIC_CUSTOM:
                this.robotDriveMode = this.robotDriveMode.fieldCentric ?
                        DriveMode.FIELD_CENTRIC_CUSTOM :
                        DriveMode.ROBOT_CENTRIC_CUSTOM;
                break;
            case FIELD_CENTRIC_FTCLIB:
            case ROBOT_CENTRIC_FTCLIB:
                this.robotDriveMode = this.robotDriveMode.fieldCentric ?
                        DriveMode.FIELD_CENTRIC_FTCLIB :
                        DriveMode.ROBOT_CENTRIC_FTCLIB;
        }

        mTelemetry.addData("RobotState: FIELD CENTRIC", this.robotDriveMode.fieldCentric);
        mTelemetry.addData("RobotState: FTCLIB DRIVE CODE", this.robotDriveMode.ftcLibDriveControl);
    }

    public enum DriveMode {
        ROBOT_CENTRIC_CUSTOM(false, false),
        FIELD_CENTRIC_CUSTOM(true, false),
        ROBOT_CENTRIC_FTCLIB(false, true),
        FIELD_CENTRIC_FTCLIB(true, true);

        public final boolean fieldCentric;
        public final boolean ftcLibDriveControl;

        private DriveMode(boolean isFieldCentric, boolean useFTCLib) {
            this.fieldCentric = isFieldCentric;
            this.ftcLibDriveControl = useFTCLib;
        }
    }
}
