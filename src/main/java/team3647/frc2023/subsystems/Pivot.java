package team3647.frc2023.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {

    private double minAngle;
    private double maxAngle;

    public Pivot(
            TalonFX master,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double minAngle,
            double maxAngle,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
    }

    public double getAngle() {
        return super.getPosition();
    }

    public void setAngle(double angle) {
        double desiredAngle = MathUtil.clamp(angle, minAngle, maxAngle);
        super.setPositionMotionMagic(desiredAngle, 0);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
