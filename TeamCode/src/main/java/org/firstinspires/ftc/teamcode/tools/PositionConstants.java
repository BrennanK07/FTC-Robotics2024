package org.firstinspires.ftc.teamcode.tools;

public class PositionConstants {
    public static class IntakeClawRotator {
        public static final double MID = 0.7168;
        public static final double CCW_MAX = 1;
        public static final double CW_MAX = 0.3729;
    }

    public static class IntakeClaw{
        public static final double CLOSED = 0.5;
        public static final double OPEN = 0.4886;
    }

    public static class VerticalClawPivot{
        public static final double LOWEST = 0.678;
        public static final double HIGHEST = 1;
    }

    public static class VerticalClaw{
        public static final double CLOSED = 0.5;
        public static final double OPEN = 0.3708;
    }

    public static class IntakePivotL{
        public static final double DOWN = 0;
        public static final double UP = 1;
        public static final double TRANSFER_POSITION = 1 - 0.325;
    }

    public static class IntakePivotR{
        public static final double DOWN = 1;
        public static final double UP = 0;
        public static final double TRANSFER_POSITION = 0.325;
    }

    public static class IntakeSlide{
        public static final int TRANSFER = 150;
        public static final int GRAB = 2408;
    }
}
