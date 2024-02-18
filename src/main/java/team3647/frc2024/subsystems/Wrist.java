package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
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
        Logger.recordOutput(
                "Intake/Pose",
                new Pose3d(
                        new Translation3d(),
                        new Rotation3d(0, -Units.degreesToRadians(getAngle()), 0)));
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
