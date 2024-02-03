package team3647.frc2024.constants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveDriveConstants {
    // default falcon rotates counter clockwise (CCW)
    // make sure gyro -CW, +CCW
    public static final SensorDirectionValue canCoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;
    public static final boolean kDriveMotorInvertedLeftSide = false;
    public static final boolean kDriveMotorInvertedRightSide = true;
    public static final boolean kTurnMotorInvertedBoolean = false;

    // physical possible max speed
    public static final double kDrivePossibleMaxSpeedMPS = 5;
    public static final double kRotPossibleMaxSpeedRadPerSec = 10;

    public static final TalonFX kFrontLeftDrive =
            new TalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftDriveId, "rio");
    public static final TalonFX kFrontLeftTurn =
            new TalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftTurnId, "rio");
    public static final CANcoder kFrontLeftAbsEncoder =
            new CANcoder(GlobalConstants.SwerveDriveIds.kFrontLeftAbsEncoderPort, "rio");

    public static final TalonFX kFrontRightDrive =
            new TalonFX(GlobalConstants.SwerveDriveIds.kFrontRightDriveId, "rio");
    public static final TalonFX kFrontRightTurn =
            new TalonFX(GlobalConstants.SwerveDriveIds.kFrontRightTurnId, "rio");
    public static final CANcoder kFrontRightAbsEncoder =
            new CANcoder(GlobalConstants.SwerveDriveIds.kFrontRightAbsEncoderPort, "rio");

    public static final TalonFX kBackLeftDrive =
            new TalonFX(GlobalConstants.SwerveDriveIds.kBackLeftDriveId, "rio");
    public static final TalonFX kBackLeftTurn =
            new TalonFX(GlobalConstants.SwerveDriveIds.kBackLeftTurnId, "rio");
    public static final CANcoder kBackLeftAbsEncoder =
            new CANcoder(GlobalConstants.SwerveDriveIds.kBackLeftAbsEncoderPort, "rio");

    public static final TalonFX kBackRightDrive =
            new TalonFX(GlobalConstants.SwerveDriveIds.kBackRightDriveId, "rio");
    public static final TalonFX kBackRightTurn =
            new TalonFX(GlobalConstants.SwerveDriveIds.kBackRightTurnId, "rio");
    public static final CANcoder kBackRightAbsEncoder =
            new CANcoder(GlobalConstants.SwerveDriveIds.kBackRightAbsEncoderPort, "rio");

    public static final Pigeon2 kGyro = new Pigeon2(GlobalConstants.SwerveDriveIds.gyroPin);

    // config swerve module reversed here, module class doens't reverse for you

    // distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(19);
    // distance between front and back wheels

    public static final double kWheelBase = Units.inchesToMeters(19);
    // translations are locations of each module wheel
    // 0 --> ++ --> front left
    // 1 --> +- --> front right
    // 2 --> -+ --> back left
    // 3 --> -- --> back right
    // c is center of robot,
    // +x towards front of robot, +y towards left of robot
    // +x
    // ^
    // |
    // +y<--c
    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    // config conversion factors here for each module. in meters for postiion and
    // radians for
    // rotation.

    // from motor to output shaft
    public static final double kDriveMotorGearRatio = 1 / 6.12;
    public static final double kTurnMotorGearRatio = 1 / 12.8; // 7.0 / 150.0;
    public static final double kCouplingGearRatio = 3.57;
    public static final double kWheelDiameterMeters = 0.097; // 97mm
    public static final double kWheelRadiusInches = 1.9;

    // // divide for tick to deg
    public static final double kTurnMotorNativeToDeg = kTurnMotorGearRatio * 360.0;

    public static final double kTurnMotorNativeToDPS = kTurnMotorNativeToDeg; // RPS / Native/10ms

    public static final double kWheelRotationToMetersDrive =
            kWheelDiameterMeters * Math.PI * kDriveMotorGearRatio;

    // Multiply by 10 because velocity is in ticks/100ms
    public static final double kFalconVelocityToMpS = kWheelRotationToMetersDrive;

    public static final double kFalconTicksToMeters = kWheelRotationToMetersDrive;

    public static final double kNominalVoltage = 10;
    public static final double kStallCurrent = 35;
    public static final double kMaxCurrent = 60;

    // comp bot
    private static final double kFrontLeftEncoderOffset = -0.738037109375;
    private static final double kFrontRightEncoderOffset = -0.18115234375;
    private static final double kBackLeftEncoderOffset = -0.618408203125;
    private static final double kBackRightEncoderOffset = -0.34228515625;

    // max speed limits that we want
    public static final double kTeleopDriveMaxAccelUnitsPerSec = kDrivePossibleMaxSpeedMPS / 2;
    public static final double kTeleopDriveMaxAngularAccelUnitsPerSec =
            kRotPossibleMaxSpeedRadPerSec / 3;

    public static final Pigeon2Configurator kGyroConfig = kGyro.getConfigurator();

    // master FF for drive for all modules
    public static final double kS = 0.22; // (0.56744 / 12); // 0.56744; // Volts
    public static final double kV = 0.47; // (2.5 / 12.0); // Volts
    public static final double kA = 0.0; // (0.0 / 12); // Volts

    public static final SimpleMotorFeedforward kMasterDriveFeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);

    // master PID constants for turn and drive for all modules
    public static final double kDriveP = 0.02; // 0.00014;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final Slot0Configs kDriveGains =
            new Slot0Configs()
                    .withKP(kDriveP)
                    .withKI(kDriveI)
                    .withKD(kDriveD)
                    .withKS(kS)
                    .withKA(kA)
                    .withKV(kV);

    public static final double kTurnP = 0.4;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0;

    public static final Slot0Configs kTurnGains =
            new Slot0Configs().withKP(kTurnP).withKI(kTurnI).withKD(kTurnD);

    public static final double kYP = 1;
    public static final double kYI = 0.0;
    public static final double kYD = 0;

    public static final PIDController kYController = new PIDController(kYP, kYI, kYD);

    public static final PIDController kAutoSteerXYPIDController = new PIDController(0.05, 0, 0);
    // 3*Pi = move at 10 rads per second if we are 180* away from target heading
    public static final PIDController kAutoSteerHeadingController = new PIDController(0.03, 0, 0);
    // PID constants for roll and yaw

    private static final SwerveModuleConstantsFactory ConstantCreator =
            new SwerveModuleConstantsFactory()
                    .withDriveMotorGearRatio(kDriveMotorGearRatio)
                    .withSteerMotorGearRatio(kTurnMotorGearRatio)
                    .withWheelRadius(kWheelRadiusInches)
                    .withSlipCurrent(300)
                    .withSteerMotorGains(kTurnGains)
                    .withDriveMotorGains(kDriveGains)
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSpeedAt12VoltsMps(kDrivePossibleMaxSpeedMPS)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withCouplingGearRatio(kCouplingGearRatio)
                    .withSteerMotorInverted(kTurnMotorInvertedBoolean);

    // is stored as reference?

    public static final SwerveDrivetrainConstants kDrivetrainConstants =
            new SwerveDrivetrainConstants()
                    .withPigeon2Id(kGyro.getDeviceID())
                    .withCANbusName("rio");

    public static final SwerveModuleConstants kFrontLeftConstants =
            ConstantCreator.createModuleConstants(
                    kFrontLeftTurn.getDeviceID(),
                    kFrontLeftDrive.getDeviceID(),
                    kFrontLeftAbsEncoder.getDeviceID(),
                    kFrontLeftEncoderOffset,
                    Units.inchesToMeters(kWheelBase / 2.0),
                    Units.inchesToMeters(kTrackWidth / 2.0),
                    kDriveMotorInvertedLeftSide);
    public static final SwerveModuleConstants kFrontRightConstants =
            ConstantCreator.createModuleConstants(
                    kFrontRightTurn.getDeviceID(),
                    kFrontRightDrive.getDeviceID(),
                    kFrontRightAbsEncoder.getDeviceID(),
                    kFrontRightEncoderOffset,
                    Units.inchesToMeters(kWheelBase / 2.0),
                    Units.inchesToMeters(-kTrackWidth / 2.0),
                    kDriveMotorInvertedRightSide);
    public static final SwerveModuleConstants kBackLeftConstants =
            ConstantCreator.createModuleConstants(
                    kBackLeftTurn.getDeviceID(),
                    kBackLeftDrive.getDeviceID(),
                    kBackLeftAbsEncoder.getDeviceID(),
                    kBackLeftEncoderOffset,
                    Units.inchesToMeters(-kWheelBase / 2.0),
                    Units.inchesToMeters(kTrackWidth / 2.0),
                    kDriveMotorInvertedLeftSide);
    public static final SwerveModuleConstants kBackRightConstants =
            ConstantCreator.createModuleConstants(
                    kBackRightTurn.getDeviceID(),
                    kBackRightDrive.getDeviceID(),
                    kBackRightAbsEncoder.getDeviceID(),
                    kBackRightEncoderOffset,
                    Units.inchesToMeters(-kWheelBase / 2.0),
                    Units.inchesToMeters(-kTrackWidth / 2.0),
                    kDriveMotorInvertedRightSide);

    private static void printError(StatusCode error) {
        if (error.value == 0) {
            return;
        }

        System.out.println(error);
    }

    static {
    }

    private SwerveDriveConstants() {}
}
