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
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final TalonFX kRightRoller =
            new TalonFX(
                    GlobalConstants.ShooterIds.kShooterRightId, GlobalConstants.subsystemsLoopName);
    public static final TalonFX kLeftRoller =
            new TalonFX(
                    GlobalConstants.ShooterIds.kShooterLeftId, GlobalConstants.subsystemsLoopName);
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kGearboxReduction = 1.5;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    // tune ff
    public static final double kS = 8.7167;
    public static final double kV = 0.24226;
    public static final double kA = 0.60231;

    public static final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double masterKP = 30;
    public static final double masterKI = 0;
    public static final double masterKD = 0;

    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 55.0;
    public static final double kMaxCurrent = 40.0;

    static {
        Slot0Configs kRightSlot0 = new Slot0Configs();
        Slot0Configs kLeftSlot0 = new Slot0Configs();

        MotorOutputConfigs kRightMotor = new MotorOutputConfigs();
        MotorOutputConfigs kLeftMotor = new MotorOutputConfigs();

        CurrentLimitsConfigs kRightCurrent = new CurrentLimitsConfigs();
        CurrentLimitsConfigs kLeftCurrent = new CurrentLimitsConfigs();

        TalonFXConfigurator kRightRollerConfigurator = kRightRoller.getConfigurator();
        TalonFXConfigurator kLeftRollerConfigurator = kLeftRoller.getConfigurator();

        kRightRollerConfigurator.apply(kMasterConfig);

        kRightMotor.NeutralMode = NeutralModeValue.Coast;
        kLeftMotor.NeutralMode = NeutralModeValue.Coast;
        kRightMotor.Inverted = InvertedValue.Clockwise_Positive;
        kLeftMotor.Inverted = InvertedValue.CounterClockwise_Positive;

        kRightSlot0.kP = masterKP;
        kRightSlot0.kI = masterKI;
        kRightSlot0.kD = masterKD;

        kLeftSlot0.kP = masterKP;
        kLeftSlot0.kI = masterKI;
        kLeftSlot0.kD = masterKD;

        kRightCurrent.StatorCurrentLimitEnable = false;
        kLeftCurrent.StatorCurrentLimitEnable = false;
        // kRightCurrent.StatorCurrentLimit = kMaxCurrent;
        // kLeftCurrent.StatorCurrentLimit = kMaxCurrent;

        kRightRollerConfigurator.apply(kRightMotor);
        kRightRollerConfigurator.apply(kRightCurrent);
        kRightRollerConfigurator.apply(kRightSlot0);

        kLeftRollerConfigurator.apply(kLeftMotor);
        kLeftRollerConfigurator.apply(kLeftCurrent);
        kLeftRollerConfigurator.apply(kLeftSlot0);
    }
}
