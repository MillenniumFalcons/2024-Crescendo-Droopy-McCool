package team3647.frc2024.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final TalonFX kRightRoller =
            new TalonFX(
                    GlobalConstants.ShooterIds.kShooterRightId, GlobalConstants.subsystemsLoopName);
    public static final TalonFX kRightSlave =
            new TalonFX(
                    GlobalConstants.ShooterIds.kShooterRightSlaveId,
                    GlobalConstants.subsystemsLoopName);
    public static final TalonFX kLeftRoller =
            new TalonFX(
                    GlobalConstants.ShooterIds.kShooterLeftId, GlobalConstants.subsystemsLoopName);
    public static final TalonFX kLeftSlave =
            new TalonFX(
                    GlobalConstants.ShooterIds.kShooterLeftSlaveId,
                    GlobalConstants.subsystemsLoopName);
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kGearboxReduction = 1.25;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.5);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    // tune ff
    public static final double kS = 0.42584; // 21.415; // 17.729; // 8.7167;
    public static final double kV = 0.13488; // 1.5; // 0.28947; // 0.24226;
    public static final double kA = 0.036548; // 0.88966; // 0.60231;

    public static final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double leftKP = 0.071466;
    public static final double rightKP = 0.071466;

    public static final double masterKI = 0;
    public static final double masterKD = 0;

    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 55.0;
    public static final double kMaxCurrent = 100.0;

    public static final double kLeftRatio = 1.65;
    public static final double kRightRatio = 2 - kLeftRatio;

    public static final InterpolatingDoubleTreeMap kLeftMap = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kRightMap = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kFeedMap = new InterpolatingDoubleTreeMap();

    static {
        kLeftMap.put(0.0, 28.0 - 6.0-0.2);
        kLeftMap.put(1.5, 28.0 - 6.0-0.2);
        kLeftMap.put(2.0, 28.0 - 4.0-0.3);
        kLeftMap.put(2.5, 28.0 - 2.0 - 0.2);
        kLeftMap.put(3.0, 28.7-0.2);
        kLeftMap.put(3.5, 1.13);

        kLeftMap.put(4.0, 28.0 + 2.2); //
        kLeftMap.put(4.5, 28.0); //
        kLeftMap.put(5.0, 28.0 + 1.4+0.8); //
        kLeftMap.put(5.5, 28.0); //
        kLeftMap.put(6.0, 28.0); //z
        kLeftMap.put(6.5, 30.0); //
        kLeftMap.put(7.0, 32.0); //
        kLeftMap.put(7.5, 34.0); //
        kLeftMap.put(8.0, 36.0); //
        kLeftMap.put(8.5, 36.0);
        kLeftMap.put(20.0, 36.0);

        kRightMap.put(0.0, 16.0 - 2.5);
        kRightMap.put(1.5, 16.0 - 2.5);
        kRightMap.put(2.0, 16.0 - 2.0);
        kRightMap.put(2.5, 16.0 - 1.0);
        kRightMap.put(3.0, 16.0);
        kRightMap.put(3.5, 16.0);
        kRightMap.put(4.0, 16.0); //
        kRightMap.put(4.5, 16.0); //
        kRightMap.put(5.0, 16.0); //
        kRightMap.put(5.5, 16.0); //
        kRightMap.put(6.0, 16.0); //
        kRightMap.put(6.5, 16.0); //
        kRightMap.put(7.0, 16.0); //
        kRightMap.put(7.5, 17.0); //
        kRightMap.put(8.0, 18.0); //
        kRightMap.put(8.5, 18.0);
        kRightMap.put(20.0, 18.0);

        kFeedMap.put(0.0, 11.0);
        kFeedMap.put(8.0, 13.0);
        kFeedMap.put(12.0, 16.4);
        kFeedMap.put(20.0, 22.4);

        Slot0Configs kRightSlot0 = new Slot0Configs();
        Slot0Configs kLeftSlot0 = new Slot0Configs();

        MotorOutputConfigs kRightMotor = new MotorOutputConfigs();
        MotorOutputConfigs kLeftMotor = new MotorOutputConfigs();

        CurrentLimitsConfigs kRightCurrent = new CurrentLimitsConfigs();
        CurrentLimitsConfigs kLeftCurrent = new CurrentLimitsConfigs();

        TalonFXConfigurator kRightRollerConfigurator = kRightRoller.getConfigurator();
        TalonFXConfigurator kLeftRollerConfigurator = kLeftRoller.getConfigurator();

        kRightRollerConfigurator.apply(kMasterConfig);

        kRightMotor.NeutralMode = NeutralModeValue.Brake;
        kLeftMotor.NeutralMode = NeutralModeValue.Brake;
        kRightMotor.PeakReverseDutyCycle = 0;
        kLeftMotor.PeakReverseDutyCycle = 0;
        kRightMotor.Inverted = InvertedValue.Clockwise_Positive;
        kLeftMotor.Inverted = InvertedValue.CounterClockwise_Positive;

        kRightSlot0.kP = rightKP;
        kRightSlot0.kI = masterKI;
        kRightSlot0.kD = masterKD;
        kRightSlot0.kS = kS;
        kRightSlot0.kV = kV;
        kRightSlot0.kA = kA;

        kLeftSlot0.kP = leftKP;
        kLeftSlot0.kI = masterKI;
        kLeftSlot0.kD = masterKD;
        kLeftSlot0.kS = kS;
        kLeftSlot0.kV = kV;
        kLeftSlot0.kA = kA;

        kRightCurrent.StatorCurrentLimitEnable = true;
        kLeftCurrent.StatorCurrentLimitEnable = true;
        kRightCurrent.StatorCurrentLimit = kMaxCurrent;
        kLeftCurrent.StatorCurrentLimit = kMaxCurrent;

        kRightRollerConfigurator.apply(kRightMotor);
        kRightRollerConfigurator.apply(kRightCurrent);
        kRightRollerConfigurator.apply(kRightSlot0);

        kLeftRollerConfigurator.apply(kLeftMotor);
        kLeftRollerConfigurator.apply(kLeftCurrent);
        kLeftRollerConfigurator.apply(kLeftSlot0);
    }
}
