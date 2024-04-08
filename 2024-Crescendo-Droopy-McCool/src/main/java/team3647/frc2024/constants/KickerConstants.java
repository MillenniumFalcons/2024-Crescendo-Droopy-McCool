package team3647.frc2024.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public class KickerConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.KickerIds.kMasterId, GlobalConstants.subsystemsLoopName);

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 25.0;
    public static final double kMaxCurrent = 20.0;

    static {
        CurrentLimitsConfigs kMasterCurrent = new CurrentLimitsConfigs();
        MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
        TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);
        kMasterCurrent.StatorCurrentLimitEnable = true;
        kMasterCurrent.StatorCurrentLimit = kMaxCurrent;
        kMasterMotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMasterMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kMasterConfigurator.apply(kMasterMotorOutput);
    }

    private KickerConstants() {}
}
