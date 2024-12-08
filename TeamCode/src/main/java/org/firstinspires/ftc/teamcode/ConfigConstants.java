package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigConstants {
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
    }

    @Config
    public static class IntakeTiming {
        public static volatile int cycleWaitAfterPiece = 10;
        public static volatile int cyclesReverse = 1;
    }

    @Config
    public static class HandoffTiming {
        public static volatile int waitBeforeEject = 50;
        public static volatile int waitAfterPiece = 10;
    }

    @Config
    public static class TestPositions {
        public static volatile double slideLeftTest = 0.5;
        public static volatile double slideRightTest = 0.5;
        public static volatile double pivotLeftTest = 0.5;
        public static volatile double pivotRightTest = 0.5;
        public static volatile double wristTest = 0.215;
        public static volatile double gripTest = 0.2;
        public static volatile double armLeftTest = 0.5;
        public static volatile double armRightTest = 0.5;
    }
}

