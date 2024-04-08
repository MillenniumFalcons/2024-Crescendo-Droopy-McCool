package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import team3647.frc2024.util.InverseKinematics;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {
    private final double minDegree;
    private final double maxDegree;
    private final double kG;

    public Wrist(
            TalonFX master,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double minDegree,
            double maxDegree,
            double nominalVoltage,
            double kG,
            double kDt) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        this.minDegree = minDegree;
        this.maxDegree = maxDegree;
        this.kG = kG;
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    @Override
    public void periodic() {
        super.periodic();
        // Logger.recordOutput(
        //         "Wrist/Pose",
        //         new Pose3d(
        //                 new Translation3d(0.28, 0, 0.18),
        //                 new Rotation3d(0, -Units.degreesToRadians(getAngle()), 0)));
        // Logger.recordOutput(
        //         "Intake/Pose",
        //         new Pose3d(
        //                 new Translation3d(),
        //                 new Rotation3d(0, -Units.degreesToRadians(getAngle()), 0)));
    }

    public double getInverseKinematics(double pivot) {
        return InverseKinematics.getWristHandoffAngleByPivot(pivot);
    }

    public double getAngle() {
        return super.getPosition();
    }

    public void setTorque(double toqrque) {
        this.setTorque(toqrque);
    }

    public void setAngle(double angle) {
        double desiredAngle = MathUtil.clamp(angle, minDegree, maxDegree);
        this.setPositionExpoVoltage(desiredAngle, 0);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
