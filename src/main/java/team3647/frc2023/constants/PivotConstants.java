package team3647.frc2023.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Transform3d;

public class PivotConstants {
    // positive is swinging towards the front of the robot
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.PivotIds.kMasterId);

    public static final boolean kMasterInvert = true;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    private static final double kGearBoxRatio = 1 / 240.0; // UPDATE THIS ONCE CAD DONE

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = kNativePosToDegrees;

    public static final double kMaxVelocityTicks = (400.0 / kNativeVelToDPS);
    public static final double kMaxAccelerationTicks = (200.0 / kNativeVelToDPS);

    public static final double kMinDegree = 0.0;
    public static final double kMaxDegree = 90.0;

    private static final double masterKP = 0.3;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double maxKG = 0.0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 30.0;
    public static final double kMaxCurrent = 60.0;

    public static final double kInitialAngle = 0;

    public static final Transform3d robotToPivot = new Transform3d();

    static {
        Slot0Configs kMasterSlot0 = new Slot0Configs();
        // VoltageConfigs kMasterVoltage = new VoltageConfigs();
        CurrentLimitsConfigs kMasterCurrent = new CurrentLimitsConfigs();
        MotionMagicConfigs kMasterMotionMagic = new MotionMagicConfigs();
        MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs kMasterSoftLimit = new SoftwareLimitSwitchConfigs();
        TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);

        kMasterSlot0.kP = masterKP;
        kMasterSlot0.kI = masterKI;
        kMasterSlot0.kD = masterKD;
        // kMasterVoltage.PeakForwardVoltage = nominalVoltage;
        // kMasterVoltage.PeakReverseVoltage = nominalVoltage;
        kMasterCurrent.StatorCurrentLimitEnable = true;
        kMasterCurrent.StatorCurrentLimit = kMaxCurrent;
        kMasterMotionMagic.MotionMagicAcceleration = kMaxVelocityTicks * 1.8;
        kMasterMotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks * 1.8;
        kMasterMotorOutput.PeakReverseDutyCycle = -0.5;
        kMasterMotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMasterMotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kMasterSoftLimit.ForwardSoftLimitEnable = true;
        kMasterSoftLimit.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;
        kMasterSoftLimit.ReverseSoftLimitEnable = true;
        kMasterSoftLimit.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;

        kMasterConfigurator.apply(kMasterSlot0);
        // kMasterConfigurator.apply(kMasterVoltage);
        kMasterConfigurator.apply(kMasterMotionMagic);
        kMasterConfigurator.apply(kMasterMotorOutput);
        kMasterConfigurator.apply(kMasterSoftLimit);
    }

    private PivotConstants() {}
}
