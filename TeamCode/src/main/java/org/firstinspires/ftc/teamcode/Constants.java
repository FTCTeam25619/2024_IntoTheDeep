package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;

public final class Constants {

    public static final class DebugModes {
        public static final boolean ENABLE_FTC_DASHBOARD = true;
    }

    public static final class RobotModes {
        public static final boolean DEBUG_TELEMETRY = false;
    }

    public static final class HardwareMapping{
        public static final String frontLeftWheel = "front_left_wheel";
        public static final String frontRightWheel = "front_right_wheel";
        public static final String backLeftWheel = "back_left_wheel";
        public static final String backRightWheel = "back_right_wheel";
        public static final String octoQuad = "octoquad";
        public static final String liftLeftMotor = "lift_left";
        public static final String liftRightMotor = "lift_right";
        public static final String liftAbsoluteEncoder = "lift_abs_enc";
        public static final String leftOdometryPod = liftLeftMotor;
        public static final String rightOdometryPod = liftRightMotor;
        public static final String perpOdometryPod = "odom_perp";
    }

    public static final class OctoQuad {
        /*
         * Identify which encoder OctoQuad input channels are connected to each sensor and what type they are.
         */
        public enum SensorType {
            MotorEncoder,
            OdometryPod;
        }

        public enum OctoQuadChannel {
            LiftLeftEncoder(0, SensorType.MotorEncoder),
            LiftRightEncoder(1, SensorType.MotorEncoder),
            // Odometry pod facing forward direction on left side of robot (Axial motion)
            OdometryLeft(2, SensorType.OdometryPod),
            // Odometry pod facing forward direction on right side of robot (Axial motion)
            OdometryRight(3, SensorType.OdometryPod),
            // Odometry pod facing perpendicular direction at the center of the robot (Lateral motion)
            OdometryPerp(4, SensorType.OdometryPod);

            public final int channelId;
            public final SensorType type;

            private OctoQuadChannel(int channelId, SensorType type) {
                this.channelId = channelId;
                this.type = type;
            }
        }
    }

    public static final class ConversionFactors {
        public static final double FEET_TO_METERS = 0.3048;
        public static final double METERS_TO_FEET = 1.0 / FEET_TO_METERS;
        public static final double DEGREES_TO_RADIANS = Math.PI / 180.0;
        public static final double RADIANS_TO_DEGREES = 180.0 / Math.PI;
        public static final double LIFT_VOLTAGE_TO_CM = 1.0;
    }

    public static final class SensorRates {
        public static final int ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS = 25;
        public static final double ODOMETRY_VELOCITY_SAMPLES_PER_SEC = 1000.0 / ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS;
    }

    public static final class DriveControl {
        // maximum theoretical drive speed
        // computed from a 5.57 ft/s theoretical number stated by GoBilda here:
        // https://www.gobilda.com/strafer-chassis-kit-104mm-gripforce-mecanum-wheels/
        public static final double THEORETICAL_MAX_DRIVE_SPEED_FPS = 5.57;
        public static final double THEORETICAL_MAX_DRIVE_SPEED_MPS = THEORETICAL_MAX_DRIVE_SPEED_FPS * ConversionFactors.FEET_TO_METERS;

        // real achievable percentage of theoretical max drive speed
        public static final double ACHIEVABLE_MAX_DRIVE_SPEED_FACTOR = 0.90;

        // joystick control deadzone for drive power (in polar coordinates) in [-1,1] range
        public static final double POWER_DEADZONE_THRESHOLD_RAW = 0.1 * Math.sqrt(2.0);

        // joystick control deadzone for turn in [-1,1] range;
        public static final double TURN_DEADZONE_THRESHOLD_RAW = 0.1;

        public static final double TURN_SPEED_LIMIT_DPS = 120.0;
        public static final double TURN_SPEED_LIMIT_RPS = TURN_SPEED_LIMIT_DPS * ConversionFactors.DEGREES_TO_RADIANS;
        public static final double SLOW_DRIVE_MODE_POWER_FACTOR = 1.8;
        public static final double SLOW_DRIVE_MODE_TURN_FACTOR = 2.5;
    }

    public static final class DriveBase {
        public static final double TRACKWIDTH_MM = 312.0;
        public static final double CENTER_WHEEL_OFFSET_MM = 168.0;
        public static final double WHEEL_RADIUS_MM = 104.0;
        public static final Translation2d FRONT_LEFT_WHEEL_POS_M =
                new Translation2d(0.207147, 0.165971);
        public static final Translation2d FRONT_RIGHT_WHEEL_POS_M =
                new Translation2d(0.207147, -0.165971);
        public static final Translation2d REAR_LEFT_WHEEL_POS_M =
                new Translation2d(-0.207147, 0.165971);
        public static final Translation2d REAR_RIGHT_WHEEL_POS_M =
                new Translation2d(-0.207147, -0.165971);
    }

    public static final class OpModes{
        public enum OpModeType{
            TELEOP,
            AUTO;
        }
        
        public enum OpModeSelection{
            DRIVE_STICKS_TELEOP(OpModeType.TELEOP);

            private OpModeType opModeType;

            private OpModeSelection(OpModeType opmodeTypeSetting){
                opModeType = opmodeTypeSetting;
            }

            public OpModeType getOpModeType(){
                return opModeType;
            }
        }
    }
}