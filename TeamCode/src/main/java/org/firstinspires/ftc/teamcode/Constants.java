package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;

public final class Constants {

    public static final class DebugModes {
        public static final boolean ENABLE_FTC_DASHBOARD = true;
        public static final boolean DEBUG_TELEMETRY = false;
    }

    public static final class HardwareMapping{
        public static final String frontLeftWheel = "front_left_wheel";
        public static final String frontRightWheel = "front_right_wheel";
        public static final String backLeftWheel = "back_left_wheel";
        public static final String backRightWheel = "back_right_wheel";
        public static final String octoQuad = "octoquad";
        public static final String climbLeftMotor = "climb_left";
        public static final String climbRightMotor = "climb_right";
        public static final String liftLeftMotor = "lift_left";
        public static final String liftRightMotor = "lift_right";
        public static final String liftAbsoluteEncoder = "lift_abs_enc";
        public static final String leftOdometryPod = liftLeftMotor;
        public static final String rightOdometryPod = liftRightMotor;
        public static final String perpOdometryPod = climbLeftMotor;
        public static final String intakePivotLeftServo = "intake_pivot_left";
        public static final String intakePivotRightServo = "intake_pivot_right";
        public static final String intakeSlideLeftServo = "intake_slide_left";
        public static final String intakeSlideRightServo = "intake_slide_right";
        public static final String intakeContinuousLeftServo = "intake_cont_left";
        public static final String intakeContinuousRightServo = "intake_cont_right";
        public static final String intakeColorSensor = "intake_color";
        public static final String depositorArmLeftServo = "depositor_arm_left";
        public static final String depositorArmRightServo = "depositor_arm_right";
        public static final String depositorWristServo = "depositor_wrist";
        public static final String depositorGripServo = "depositor_grip";
        public static final String depositorColorSensor = "depositor_color";
        public static final String sweepServo = "sweep";
        public static final String leds = "leds";
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
            ClimbLeftEncoder(6, SensorType.MotorEncoder),
            ClimbRightEncoder(7, SensorType.MotorEncoder),
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
    }

    public static final class Sensors {
        public static final int ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS = 25;
        public static final double ODOMETRY_VELOCITY_SAMPLES_PER_SEC = 1000.0 / ODOMETRY_VELOCITY_SAMPLE_INTERVAL_MS;
        public static final double COLOR_MATCH_DISTANCE_THRESHOLD = 0.8;
    }

    public static final class Lift {
        public static final double MAX_V = 3.070;
        public static final double MIN_V = 1.680;
        public static final double V_RANGE = MAX_V - MIN_V;
        public static final double POT_ROTATIONS = 10.0;
        public static final double SPOOL_CIRC_CM = 7.5819;
        public static final double SENSOR_MAX_CM = 71.755;
        public static final double LIFT_V_TO_CM = SENSOR_MAX_CM / V_RANGE;
        public enum MovementDirection {
            NONE,
            UP,
            DOWN;
        }
    }

    public static final class DriveControl {
        // maximum theoretical drive speed
        // computed from a 5.57 ft/s theoretical number stated by GoBilda here:
        // https://www.gobilda.com/strafer-chassis-kit-104mm-gripforce-mecanum-wheels/
        public static final double THEORETICAL_MAX_DRIVE_SPEED_FPS = 5.57;
        public static final double THEORETICAL_MAX_DRIVE_SPEED_MPS = THEORETICAL_MAX_DRIVE_SPEED_FPS * ConversionFactors.FEET_TO_METERS;

        // real achievable percentage of theoretical max drive speed
        public static final double ACHIEVABLE_MAX_DRIVE_SPEED_FACTOR = 0.90;

        public static final double TURN_SPEED_LIMIT_DPS = 120.0;
        public static final double TURN_SPEED_LIMIT_RPS = TURN_SPEED_LIMIT_DPS * ConversionFactors.DEGREES_TO_RADIANS;
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

    public static final class Intake {
        public static enum SlideSetPosition {
            IN(0.15, 0.85),
            NEUTRAL(0.20, 0.80),
            OUT_NEAR(0.275, 0.725),
            OUT_FAR(0.427, 0.573);

            public final double leftPosition;
            public final double rightPosition;

            private SlideSetPosition(double leftPosition, double rightPosition) {
                this.leftPosition = leftPosition;
                this.rightPosition = rightPosition;
            }
        }

        public static enum PivotSetPosition {
            UP(0.985, 0.01),
            DOWN(0.0, 1.0);

            public final double leftPosition;
            public final double rightPosition;

            private PivotSetPosition(double leftPosition, double rightPosition) {
                this.leftPosition = leftPosition;
                this.rightPosition = rightPosition;
            }
        }

        public static double leftSlideRange = Constants.Intake.SlideSetPosition.OUT_NEAR.leftPosition -
                Constants.Intake.SlideSetPosition.OUT_FAR.leftPosition;
        public static final double rightSlideRange = Constants.Intake.SlideSetPosition.OUT_NEAR.rightPosition -
                Constants.Intake.SlideSetPosition.OUT_FAR.rightPosition;
        public static final double rightToLeftSlideRatio = rightSlideRange / leftSlideRange;
    }

    public static final class Depositor {
        public static enum ArmSetPosition {
            HOME(0.0, 0.985),
            NEUTRAL(0.02, 0.945),
            SCORING(0.8, 0.175);

            public final double leftPosition;
            public final double rightPosition;

            private ArmSetPosition(double leftPosition, double rightPosition) {
                this.leftPosition = leftPosition;
                this.rightPosition = rightPosition;
            }
        }

        public static enum WristSetPosition {
            INTAKE(0.215),
            SCORING(0.715);

            public final double position;

            private WristSetPosition(double position) {
                this.position = position;
            }
        }

        public static enum GripSetPosition {
            OPEN(0.2),
            CLOSED(0.7);

            public final double position;

            private GripSetPosition(double position) {
                this.position = position;
            }
        }

        public static enum SweepSetPosition {
            HOME(0.005),
            SWEEP(0.05);

            public final double position;

            private SweepSetPosition(double position) {
                this.position = position;
            }
        }
    }

    public static enum ScoringPosition {
        LOW_BASKET(ConfigConstants.Lift.LIFT_LOW_BASKET),
        HIGH_BASKET(ConfigConstants.Lift.LIFT_HI_BASKET),
        HIGH_CLIP_SCORE(ConfigConstants.Lift.LIFT_HI_CLIP_SCORE),
        HIGH_CLIP(ConfigConstants.Lift.LIFT_HI_CLIP);

        public final double liftPosition;

        private ScoringPosition(double liftPosition) {
            this.liftPosition = liftPosition;
        }
    }

    public static final class OpModes{
        public enum OpModeType{
            TELEOP,
            AUTO;
        }
        
        public enum OpModeSelection{
            DISSECTION(OpModeType.TELEOP),
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