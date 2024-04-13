package team3647.frc2024.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public class ClimbConstants {
    public static final TalonFX kLeft =
            new TalonFX(GlobalConstants.ClimbIds.kLeftId, GlobalConstants.subsystemsLoopName);
    public static final TalonFX kRight =
            new TalonFX(GlobalConstants.ClimbIds.kRightId, GlobalConstants.subsystemsLoopName);

    public static final boolean kMasterInvert = false;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    private static final double kGearBoxRatio = 1 / 18;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = kNativePosToDegrees;

    public static final double kMaxVelocityTicks = (400.0 / kNativeVelToDPS);
    public static final double kMaxAccelerationTicks = (200.0 / kNativeVelToDPS);

    public static final double kMinLength = 0;
    public static final double kMaxDegreeLength = 61.2;

    private static final double masterKP = 1.5;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double maxKG = 0.0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 30.0;
    public static final double kMaxCurrent = 40.0;

    public static final double kInitialLength = 0;

    static {
        Slot0Configs kMasterSlot0 = new Slot0Configs();
        MotorOutputConfigs kLeftMotorOutput = new MotorOutputConfigs();
        MotorOutputConfigs kRightMotorOutput = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs kMasterSoftLimit = new SoftwareLimitSwitchConfigs();
        TalonFXConfigurator kLeftConfigurator = kLeft.getConfigurator();
        TalonFXConfigurator kRightConfigurator = kRight.getConfigurator();
        kLeftConfigurator.apply(kMasterConfig);
        kRightConfigurator.apply(kMasterConfig);

        kMasterSlot0.kP = masterKP;
        kMasterSlot0.kI = masterKI;
        kMasterSlot0.kD = masterKD;
        kLeftMotorOutput.NeutralMode = NeutralModeValue.Brake;
        kLeftMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kRightMotorOutput.NeutralMode = NeutralModeValue.Brake;
        kRightMotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kMasterSoftLimit.ForwardSoftLimitEnable = true;
        kMasterSoftLimit.ForwardSoftLimitThreshold = 80;
        kMasterSoftLimit.ReverseSoftLimitEnable = true;
        kMasterSoftLimit.ReverseSoftLimitThreshold = 0;

        kLeftConfigurator.apply(kMasterSlot0);
        kLeftConfigurator.apply(kLeftMotorOutput);
        kLeftConfigurator.apply(kMasterSoftLimit);
        kRightConfigurator.apply(kMasterSlot0);
        kRightConfigurator.apply(kRightMotorOutput);
        kRightConfigurator.apply(kMasterSoftLimit);
    }

    private ClimbConstants() {}
}
