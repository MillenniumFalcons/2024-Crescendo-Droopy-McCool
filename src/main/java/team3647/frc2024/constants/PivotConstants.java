package team3647.frc2024.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class PivotConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.PivotIds.kMasterId, GlobalConstants.subsystemsLoopName);
    public static final TalonFX kSlave =
            new TalonFX(GlobalConstants.PivotIds.kSlaveId, GlobalConstants.subsystemsLoopName);

    public static final boolean kMasterInvert = false;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    private static final double kGearBoxRatio = 1 / 34.72;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = kNativePosToDegrees;

    public static final double kMaxVelocityTicks = (400.0 / kNativeVelToDPS);
    public static final double kMaxAccelerationTicks = (200.0 / kNativeVelToDPS);

    public static final double kMinDegree = 15;
    public static final double kMaxDegree = 61.2;
    public static final double kMaxDegreeUnderStage = 30;

    private static final double masterKP = 1.5;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double maxKG = 0.0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 30.0;
    public static final double kMaxCurrent = 40.0;

    public static final double kInitialAngle = 61.2;

    public static final Transform3d robotToPivot =
            new Transform3d(
                    new Translation3d(Units.inchesToMeters(0.3), 0, Units.inchesToMeters(17.2)),
                    new Rotation3d());

    public static final TimeOfFlight tofBack =
            new TimeOfFlight(GlobalConstants.SensorIds.pivotBackId);
    public static final TimeOfFlight tofFront =
            new TimeOfFlight(GlobalConstants.SensorIds.pivotFrontId);

    // distance squared vs pivot angle
    public static final InterpolatingDoubleTreeMap kMasterSpeakerMap =
            new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kMasterAmpMap = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kMasterTrapMap =
            new InterpolatingDoubleTreeMap();

    static {
        kMasterSpeakerMap.put(0.0, 60.0);
        kMasterSpeakerMap.put(1.26, 60.0);
        kMasterSpeakerMap.put(2.0, 52.0);
        kMasterSpeakerMap.put(2.5, 43.0);
        kMasterSpeakerMap.put(3.0, 37.5);
        kMasterSpeakerMap.put(3.5, 34.0);
        kMasterSpeakerMap.put(4.0, 31.0);
        kMasterSpeakerMap.put(4.5, 28.1);
        kMasterSpeakerMap.put(5.0, 26.2);
        kMasterSpeakerMap.put(5.5, 22.5);
        kMasterSpeakerMap.put(6.0, 20.0);
        kMasterSpeakerMap.put(20.0, 17.0);
        kMasterAmpMap.put(0.0, 60.0);
        kMasterAmpMap.put(100.0, 60.0);
        Slot0Configs kMasterSlot0 = new Slot0Configs();
        CurrentLimitsConfigs kMasterCurrent = new CurrentLimitsConfigs();
        MotionMagicConfigs kMasterMotionMagic = new MotionMagicConfigs();
        MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs kMasterSoftLimit = new SoftwareLimitSwitchConfigs();
        TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        TalonFXConfigurator kSlaveConfigurator = kSlave.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);

        kMasterSlot0.kP = masterKP;
        kMasterSlot0.kI = masterKI;
        kMasterSlot0.kD = masterKD;
        kMasterCurrent.StatorCurrentLimitEnable = true;
        kMasterCurrent.StatorCurrentLimit = kMaxCurrent;
        kMasterMotionMagic.MotionMagicAcceleration = kMaxVelocityTicks * 1.8;
        kMasterMotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks * 1.8;
        kMasterMotionMagic.MotionMagicExpo_kA = 0.06;
        kMasterMotionMagic.MotionMagicExpo_kV = 0.09;
        kMasterMotorOutput.PeakReverseDutyCycle = -0.5;
        kMasterMotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMasterMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kMasterSoftLimit.ForwardSoftLimitEnable = true;
        kMasterSoftLimit.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;
        kMasterSoftLimit.ReverseSoftLimitEnable = true;
        kMasterSoftLimit.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;

        kMasterConfigurator.apply(kMasterSlot0);
        kMasterConfigurator.apply(kMasterMotionMagic);
        kMasterConfigurator.apply(kMasterMotorOutput);
        kMasterConfigurator.apply(kMasterSoftLimit);
        kSlaveConfigurator.apply(kMasterSlot0);
        kSlaveConfigurator.apply(kMasterMotionMagic);
        kSlaveConfigurator.apply(kMasterMotorOutput);
        kSlaveConfigurator.apply(kMasterSoftLimit);
    }

    private PivotConstants() {}
}
