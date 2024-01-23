package team3647.frc2023.constants;

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
    public static final TalonFX kTopRoller = new TalonFX(GlobalConstants.ShooterIds.kShooterTopId);
    public static final TalonFX kBottomRoller =
            new TalonFX(GlobalConstants.ShooterIds.kShooterBottomId);
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kGearboxReduction = 2.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(2);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    public static final double kS = 0.23;
    public static final double kV = 2.6;
    public static final double kA = 0.0;

    public static final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double masterKP = 0.03;
    public static final double masterKI = 0;
    public static final double masterKD = 0;

    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 25.0;
    public static final double kMaxCurrent = 20.0;

    static {
        Slot0Configs kTopSlot0 = new Slot0Configs();
        Slot0Configs kBottomSlot0 = new Slot0Configs();

        MotorOutputConfigs kTopMotor = new MotorOutputConfigs();
        MotorOutputConfigs kBottomMotor = new MotorOutputConfigs();

        CurrentLimitsConfigs kTopCurrent = new CurrentLimitsConfigs();
        CurrentLimitsConfigs kBottomCurrent = new CurrentLimitsConfigs();

        TalonFXConfigurator kTopRollerConfigurator = kTopRoller.getConfigurator();
        TalonFXConfigurator kBottomRollerConfigurator = kBottomRoller.getConfigurator();

        kTopRollerConfigurator.apply(kMasterConfig);

        kTopMotor.NeutralMode = NeutralModeValue.Brake;
        kBottomMotor.NeutralMode = NeutralModeValue.Brake;
        kTopMotor.Inverted = InvertedValue.CounterClockwise_Positive;
        kBottomMotor.Inverted = InvertedValue.CounterClockwise_Positive;

        kTopSlot0.kP = masterKP;
        kTopSlot0.kI = masterKI;
        kTopSlot0.kD = masterKD;

        kBottomSlot0.kP = masterKP;
        kBottomSlot0.kI = masterKI;
        kBottomSlot0.kD = masterKD;

        kTopCurrent.StatorCurrentLimitEnable = false;
        kBottomCurrent.StatorCurrentLimitEnable = false;

        kTopRollerConfigurator.apply(kTopMotor);
        kTopRollerConfigurator.apply(kTopCurrent);
        kTopRollerConfigurator.apply(kTopSlot0);

        kBottomRollerConfigurator.apply(kTopMotor);
        kBottomRollerConfigurator.apply(kBottomCurrent);
        kBottomRollerConfigurator.apply(kBottomSlot0);
    }
}
