package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigConstants {
    @Config
    public static class Climb {
        public static volatile double kPLeft = 0.1;
        public static volatile double kILeft = 0.0;
        public static volatile double kDLeft = 0.0;
        public static volatile double kFLeft = 0.1;
        public static volatile double kPRight = 0.1;
        public static volatile double kIRight = 0.0;
        public static volatile double kDRight = 0.0;
        public static volatile double kFRight = 0.1;
        public static volatile double pidTolerance = 2000.0;
        public static volatile int holdTimeoutMS = 20000;
        public static volatile double maxHoldMotorPower = 0.1;
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
        public static volatile double climbUpMotorPower = 1.0;
        public static volatile double climbDownMotorPower = -1.0;
    }

    @Config
    public static class IntakeTiming {
        public static volatile int cycleWaitAfterPiece = 10;
        public static volatile int cyclesReverse = -1;
        public static volatile double spinSpeedFactor = 1.0;
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
    public static class HandoffTiming {
        public static volatile int waitBeforeEject = 50;
        public static volatile int waitAfterPiece = 10;
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
    }
}

