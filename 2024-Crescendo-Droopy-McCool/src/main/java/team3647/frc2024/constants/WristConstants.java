package team3647.frc2024.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.playingwithfusion.TimeOfFlight;

public final class WristConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.WristIds.kMasterId, GlobalConstants.subsystemsLoopName);

    private static final double kGearBoxRatio = 1.0 / 37.50;
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNativePosToDegrees = kGearBoxRatio * 360.0;

    public static final double kNativeVelToDPS = kNativePosToDegrees;

    private static final double masterKP = 1.8;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 20.0;
    public static final double kMaxCurrent = 15;

    public static final double kG = 0.0;

    public static final double kMaxVelocityTicks = (600.0 / kNativeVelToDPS) * 8;
    public static final double kMaxAccelerationTicks = (300.0 / kNativeVelToDPS) * 8;

    public static final double kMinDegree = 0;
    public static final double kMaxDegree = 131.5; // 131.5 // 76.4325;

    public static final double kInitialDegree = 131.5; // 131.5

    public static final TimeOfFlight tof = new TimeOfFlight(GlobalConstants.SensorIds.wristId);

    static {
        Slot0Configs kMasterSlot0 = new Slot0Configs();
        VoltageConfigs kMasterVoltage = new VoltageConfigs();
        CurrentLimitsConfigs kMasterCurrent = new CurrentLimitsConfigs();
        MotionMagicConfigs kMasterMotionMagic = new MotionMagicConfigs();
        MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs kMasterSoftLimit = new SoftwareLimitSwitchConfigs();
        TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);

        kMasterSlot0.kP = masterKP;
        kMasterSlot0.kI = masterKI;
        kMasterSlot0.kD = masterKD;
        kMasterCurrent.StatorCurrentLimitEnable = true;
        kMasterCurrent.StatorCurrentLimit = kMaxCurrent;
        kMasterMotionMagic.MotionMagicAcceleration = kMaxVelocityTicks;
        kMasterMotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks;
        kMasterMotionMagic.MotionMagicExpo_kA = 0.01;
        kMasterMotionMagic.MotionMagicExpo_kV = 0.01;
        kMasterMotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMasterMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kMasterSoftLimit.ForwardSoftLimitEnable = true;
        kMasterSoftLimit.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;
        kMasterSoftLimit.ReverseSoftLimitEnable = true;
        kMasterSoftLimit.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;

        kMasterConfigurator.apply(kMasterSlot0);
        kMasterConfigurator.apply(kMasterVoltage);
        kMasterConfigurator.apply(kMasterMotionMagic);
        kMasterConfigurator.apply(kMasterMotorOutput);
        kMasterConfigurator.apply(kMasterSoftLimit);
    }

    private WristConstants() {}
}
