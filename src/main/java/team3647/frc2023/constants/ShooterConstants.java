package team3647.frc2023.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Transform3d;

public class ShooterConstants {
    public static final TalonFX kTopRoller = new TalonFX(GlobalConstants.ShooterIds.kShooterTopId);
    public static final TalonFX kBottomRoller =
            new TalonFX(GlobalConstants.ShooterIds.kShooterBottomId);
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final boolean kTopMotorInvert = false;
    public static final boolean kBottomMotorInvert = false;

    // kG at max extension
    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 25.0;
    public static final double kMaxCurrent = 20.0;

    public static final Transform3d robotToShooter = new Transform3d();

    static {
        MotorOutputConfigs kTopMotor = new MotorOutputConfigs();
        MotorOutputConfigs kBottomMotor = new MotorOutputConfigs();

        // VoltageConfigs kTopVoltage = new VoltageConfigs();
        // VoltageConfigs kBottomVoltage = new VoltageConfigs();

        CurrentLimitsConfigs kTopCurrent = new CurrentLimitsConfigs();
        CurrentLimitsConfigs kBottomCurrent = new CurrentLimitsConfigs();

        TalonFXConfigurator kTopRollerConfigurator = kTopRoller.getConfigurator();
        TalonFXConfigurator kBottomRollerConfigurator = kBottomRoller.getConfigurator();

        kTopRollerConfigurator.apply(kMasterConfig);

        kTopMotor.NeutralMode = NeutralModeValue.Brake;
        kBottomMotor.NeutralMode = NeutralModeValue.Brake;
        kTopMotor.Inverted = InvertedValue.Clockwise_Positive;
        kBottomMotor.Inverted = InvertedValue.Clockwise_Positive;

        kTopCurrent.StatorCurrentLimitEnable = false;
        kBottomCurrent.StatorCurrentLimitEnable = false;
        // kTopCurrent.StatorCurrentLimit = kMaxCurrent;
        // kBottomCurrent.StatorCurrentLimit = kMaxCurrent;

        kTopRollerConfigurator.apply(kTopMotor);
        kTopRollerConfigurator.apply(kTopCurrent);

        kBottomRollerConfigurator.apply(kTopMotor);
        // kBottomRollerConfigurator.apply(kTopVoltage);
        kBottomRollerConfigurator.apply(kBottomCurrent);
    }
}
