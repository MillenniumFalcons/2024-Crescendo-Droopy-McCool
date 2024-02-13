package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {

    private final double minAngle;
    private double maxAngle;
    private final double maxAngleNormal;
    private final double maxAngleUnderStage;

    private final Supplier<Pose2d> drivePose;

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
            Supplier<Pose2d> drivePose,
            double nominalVoltage,
            double maxKG,
            double kDt,
            TimeOfFlight tofBack,
            TimeOfFlight tofFront) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        super.addFollower(slave, false);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.maxAngleNormal = maxAngle;
        this.maxAngleUnderStage = maxAngleUnderStage;
        this.drivePose = drivePose;
        this.maxKG = maxKG;
        this.tofBack = tofBack;
        this.tofFront = tofFront;
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (((drivePose.get().getX() > 3.5 && drivePose.get().getX() < 6.2)
                        || (drivePose.get().getX() > 10.2 && drivePose.get().getX() < 12.9))
                && ((Math.abs(drivePose.get().getY() - 4)
                                        < ((drivePose.get().getX() - 2.9) * 1 / 1.73)
                                && drivePose.get().getX() < 6.2)
                        || (Math.abs(drivePose.get().getY() - 4)
                                        < ((13.6 - drivePose.get().getX()) * 1 / 1.73)
                                && drivePose.get().getX() > 10.2))) {
            this.maxAngle = maxAngleUnderStage;
        } else {
            this.maxAngle = maxAngleNormal;
        }
        Logger.recordOutput(
                "Pivot/Pose",
                new Pose3d(
                        new Translation3d(),
                        new Rotation3d(0, Units.degreesToRadians(getAngle()), 0)));
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
        var ffvolts = maxKG * Math.cos(desiredAngle);
        super.setPositionMotionMagic(desiredAngle, ffvolts);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    public boolean backPiece() {
        return tofBack.getRange() < 100;
    }

    public boolean frontPiece() {
        return tofFront.getRange() < 100;
    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
