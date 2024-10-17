package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static final class HardwareMapping{
        public static final String frontLeftWheel = "front_left_wheel";
        public static final String frontRightWheel = "front_right_wheel";
        public static final String backLeftWheel = "back_left_wheel";
        public static final String backRightWheel = "back_right_wheel";

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