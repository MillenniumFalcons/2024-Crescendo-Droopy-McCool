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
    public static final TalonFX kRightRoller = new TalonFX(GlobalConstants.ShooterIds.kShooterRightId);
    public static final TalonFX kLeftRoller =
            new TalonFX(GlobalConstants.ShooterIds.kShooterLeftId);
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kGearboxReduction = 2.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(2);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

            // tune ff
    public static final double kS = 0.23;
    public static final double kV = 2.6;
    public static final double kA = 0.0;

    public static final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double masterKP = 0.03;
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

        kRightMotor.NeutralMode = NeutralModeValue.Brake;
        kLeftMotor.NeutralMode = NeutralModeValue.Brake;
        kRightMotor.Inverted = InvertedValue.CounterClockwise_Positive;
        kLeftMotor.Inverted = InvertedValue.Clockwise_Positive;

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

        kLeftRollerConfigurator.apply(kRightMotor);
        kLeftRollerConfigurator.apply(kLeftCurrent);
        kLeftRollerConfigurator.apply(kLeftSlot0);
    }
}
