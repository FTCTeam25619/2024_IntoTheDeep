package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigConstants {
    @Config
    public static class Climb {
        public static volatile double kP = 0.0001;
        public static volatile double kI = 0.0;
        public static volatile double kD = 0.0;
        public static volatile double kF = 0.0;
        public static volatile double SYNC_KP = 0.0001;
        public static volatile double SYNC_KI = 0.0;
        public static volatile double SYNC_KD = 0.0;
        public static volatile double pidTolerance = 500.0;
        public static volatile int holdTimeoutMS = 600000;
        public static volatile double maxHoldMotorPower = 0.1;
        public static volatile int movementBufferCounts = 400;

    }

    @Config
    public static class Colors {
        public static float BLUE_HUE_MIN = 210;
        public static float BLUE_HUE_MAX = 250;
        public static float RED_HUE_MIN = 10;
        public static float RED_HUE_MAX = 40;
        public static float YELLOW_HUE_MIN = 65;
        public static float YELLOW_HUE_MAX = 95;
        public static float BLACK_HUE_MIN = 0;
        public static float BLACK_HUE_MAX = 10;
    }

    @Config
    public static class ManualMovement {
        public static volatile double liftDownMotorPower = -0.15;
        public static volatile double liftUpMotorPower = 0.5;
        public static volatile double climbUpMotorPower = -1.0;
        public static volatile double climbDownMotorPower = 1.0;
        public static volatile double slideManualIncrementExtend = 0.03;
        public static volatile double slideManualIncrementRetract = -0.03;
        public static volatile double slideManualDeadband = 0.1;
        public static volatile double slideManualThreshold = 0.1;
    }

    @Config
    public static class IntakeTiming {
        public static volatile int cycleWaitAfterPiece = 10;
        public static volatile int cyclesReverse = -1;
        public static volatile double spinSpeedFactor = 1.0;
        public static volatile int extendWaitForPivotingMS = 100;
        public static volatile int handoffWaitForMateMS = 300;
        public static volatile int handoffWaitBeforeSlideMove = 620;
        public static volatile double handoffSpeedFactor = 1.0;
    }

    @Config
    public static class ScoringTiming {
        public static volatile int preScoreWaitForLiftMS = 250;
        public static volatile int preScoreWaitForArmMS = 0;
        public static volatile int preScoreWaitForWristMS = 550;
        public static volatile int postScoreWaitForArmMS = 500;
        public static volatile int postScoreWaitForWristMS = 250;
        public static volatile int postScoreWaitForLiftMS = 500;
        public static volatile int postScoreWaitForHomeMS = 1000;
    }

    @Config
    public static class Lift {
        // TODO:  The following are placeholder values!  Update them.
        public static volatile double MIN_POS_CM = 0.0;
        public static volatile double MAX_POS_CM = 70.0;

        public static volatile double LIFT_DOWN_POS = MIN_POS_CM + 0.5;  //Add small tolerance to avoid hard limit
        public static volatile double LIFT_LOW_BASKET = 21.0;
        public static volatile double LIFT_HI_BASKET = 62.0;
        public static volatile double LIFT_SPECIMEN_HIGH = 6.7;
        public static volatile double LIFT_SPECIMEN_SCORE = 6.7;

        public static volatile double MAX_UP_POWER = 1.0;
        public static volatile double MAX_UP_POWER_CLOSE = 0.4;
        public static volatile double MAX_DOWN_POWER = -0.6;
        public static volatile double MAX_DOWN_POWER_CLOSE = -0.25;
        public static volatile double TARGET_SLOW_ZONE_THRESHOLD = 5.0;
        public static volatile double HOME_SLOW_ZONE_THRESHOLD = 3.0;
        public static volatile double LIFT_TOLERANCE_UP = 0.5;  // +/- CM
        public static volatile double LIFT_TOLERANCE_DOWN = 0.2;  // +/- CM

        @Config
        public static class LiftPID {
            public static volatile double kPUp = 0.07;  // Max power at 2cm error
            public static volatile double kIUp = 0.0;
            public static volatile double kDUp = 0.0125;
            public static volatile double kFUp = 0.25;   // % motor power to counteract gravity
            public static volatile double kPDown = 0.35;  // Max power at 2cm error
            public static volatile double kIDown = 0.0;
            public static volatile double kDDown = 0.0;
            public static volatile double kFDown = 0.0;   // % motor power to counteract gravity
        }

    }

    @Config
    public static class DriveControl {

        // joystick control deadzone for drive power (in polar coordinates) in [-1,1] range
        public static volatile double POWER_DEADZONE_THRESHOLD_RAW = 0.1 * Math.sqrt(2.0);

        // joystick control deadzone for turn in [-1,1] range;
        public static volatile double TURN_DEADZONE_THRESHOLD_RAW = 0.1;

        public static volatile double SLOW_DRIVE_MODE_POWER_FACTOR = 1.8;
        public static volatile double SLOW_DRIVE_MODE_TURN_FACTOR = 4.0;

        public static volatile double kP_ORIENTATION = 0.01;
    }

    @Config
    public static class TestPositions {
        public static volatile double slideLeftTest = 0.23;
        public static volatile double slideRightTest = 0.015;
        public static volatile double pivotLeftTest = 0.985;
        public static volatile double pivotRightTest = 0.01;
        public static volatile double wristTest = 0.215;
        public static volatile double gripTest = 0.2;
        public static volatile double armLeftTest = 0.8;
        public static volatile double armRightTest = 0.175;
        public static volatile double sweepTest = 0.0;
    }
}

