package team3647.frc2024.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class GlobalConstants {
    public static final double kFalconTicksPerRotation = 1;
    public static final double kFalcon5TicksPerRotation = 2048;
    public static final double kDt = 0.02;
    public static final int kTimeoutMS = 255;

    public static final String subsystemsLoopName = "subsystems";

    public static final class SwerveDriveIds {
        public static final int kFrontLeftDriveId = 1;
        public static final int kFrontLeftTurnId = 2;
        public static final int kFrontLeftAbsEncoderPort = 9;

        public static final int kFrontRightDriveId = 3;
        public static final int kFrontRightTurnId = 4;
        public static final int kFrontRightAbsEncoderPort = 10;

        public static final int kBackLeftDriveId = 5;
        public static final int kBackLeftTurnId = 6;
        public static final int kBackLeftAbsEncoderPort = 11;

        public static final int kBackRightDriveId = 7;
        public static final int kBackRightTurnId = 8;
        public static final int kBackRightAbsEncoderPort = 12;

        public static final int gyroPin = 16;

        // gear Ratio: 1:6.12(motor:output shaft) (L3 mk4 sds modules)
        // wheel diameter: 4 in | meters per motor rotation: 0.3191858136m (4 inch
        // diameter wheels * pi)x

        private SwerveDriveIds() {}
    }

    public static final class PivotIds {
        public static final int kMasterId = 20;
        public static final int kSlaveId = 21;
    }

    public static final class ShooterIds {
        public static final int kShooterLeftId = 22;
        public static final int kShooterRightId = 23;
        public static final int kShooterLeftSlaveId = 33;
        public static final int kShooterRightSlaveId = 34;
    }

    public static final class KickerIds {
        public static final int kMasterId = 24;
    }

    public static final class WristIds {
        public static final int kMasterId = 25;
    }

    public static final class IntakeIds {
        public static final int kMasterId = 26;
    }

    public static final class ClimbIds {
        public static final int kLeftId = 27;
        public static final int kRightId = 28;
    }

    public static final class ChurroIds {
        public static final int kMasterId = 32;
    }

    public static final class SensorIds {
        public static final int wristId = 29;
        public static final int pivotBackId = 30;
        public static final int pivotFrontId = 31;
    }

    private GlobalConstants() {}
}
