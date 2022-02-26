package frc.robot;

public class RobotMap {
    public final static class Motors {
        public final static int frontJack = 5;
        public final static int rearJack = 8;

        public final static int FLDrive = 7;
        public final static int BLDrive = 12;
        public final static int FRDrive = 6;
        public final static int BRDrive = 11;

        public final static int armMotor = 10;
        public final static int wrist = 9; 
        public final static int intake = 14;

        public final static int jackWheel = 13;
    }

    public final static class limitSwitches {
        public final static int armDown = 5;
        public final static int armUp = 2;

        public final static int rearJackUp = 0;
        public final static int rearJackDown = 4;

        public final static int wristDown = 1;
        public final static int wristUp = 3;

        public final static int frontjackUp = 6;
        public final static int frontJackDown = 7;
    }

    public final static class Encoders {
        public final static int armA = 8;
        public final static int armB = 9;

        public final static int wrist = 18;
    }

}   