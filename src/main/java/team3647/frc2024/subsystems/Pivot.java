package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {

    private final double minAngle;
    private double maxAngle;
    private final double maxAngleNormal;
    private final double maxAngleUnderStage;

    BooleanSupplier underStage;
    BooleanSupplier isIntaking;

    private final double maxKG;

    private Pose3d pose = new Pose3d(new Translation3d(), new Rotation3d(0, 0, 0));

    private final TimeOfFlight tofBack;
    private final TimeOfFlight tofFront;

    public Pivot(
            TalonFX master,
            TalonFX slave,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double minAngle,
            double maxAngle,
            double maxAngleUnderStage,
            BooleanSupplier underStage,
            double nominalVoltage,
            double maxKG,
            double kDt,
            TimeOfFlight tofBack,
            TimeOfFlight tofFront,
            BooleanSupplier isIntaking) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        super.addFollower(slave, false);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.maxAngleNormal = maxAngle;
        this.maxAngleUnderStage = maxAngleUnderStage;
        this.underStage = underStage;
        this.maxKG = maxKG;
        this.tofBack = tofBack;
        this.tofFront = tofFront;
        this.isIntaking = isIntaking;

        master.getPosition().setUpdateFrequency(250);
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    @Override
    public void periodic() {
        super.periodic();
        // Logger.recordOutput("fornt tof", tofFront());
        Logger.recordOutput("front tof", tofFront());
        // Logger.recordOutput("pibotdimand", motionMagicExpoVoltage.Position * positionConversion);

        // Logger.recordOutput(
        //         "Pivot/Pose",
        //         new Pose3d(
        //                 new Translation3d(
        //                         Units.inchesToMeters(0.3), 0, Units.inchesToMeters(17.2 + 1.75)),
        //                 new Rotation3d(0, Units.degreesToRadians(getAngle()), 0)));
        if (underStage.getAsBoolean()) {
            this.maxAngle = maxAngleUnderStage;
        } else {
            this.maxAngle = maxAngleNormal;
        }
        //     Logger.recordOutput(
        //             "Pivot/Pose",
        //             new Pose3d(
        //                     new Translation3d(),
        //                     new Rotation3d(0, Units.degreesToRadians(getAngle()), 0)));
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
    }

    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    public double getAngle() {
        return super.getPosition();
    }

    public void setAngle(double angle) {
        double desiredAngle = MathUtil.clamp(angle, minAngle, maxAngle);
        super.setPositionExpoVoltage(desiredAngle, 0);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    public boolean backPiece() {
        return tofBack.getRange() < 125;
    }

    public double tofBack() {
        return tofBack.getRange();
    }

    public boolean frontPiece() {
        return tofFront.getRange() < 80;
    }

    public double tofFront() {
        return tofFront.getRange();
    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
