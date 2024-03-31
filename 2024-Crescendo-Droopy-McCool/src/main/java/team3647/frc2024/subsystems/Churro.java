package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import team3647.lib.TalonFXSubsystem;

public class Churro extends TalonFXSubsystem {
    private final double minDegree;
    private final double maxDegree;
    private final double kG;

    public Churro(
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

    public void setAngleSpringy(double angle) {
        double desiredAngle = MathUtil.clamp(angle, minDegree, maxDegree);
        this.setPositionExpoVoltage(desiredAngle, 0, 1);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        return "Churro";
    }
}
