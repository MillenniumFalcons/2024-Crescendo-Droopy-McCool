package team3647.frc2023.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {

    private double minAngle;
    private double maxAngle;

    private double maxKG;

    public Pivot(
            TalonFX master,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double minAngle,
            double maxAngle,
            double nominalVoltage,
            double maxKG,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.maxKG = maxKG;
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
        Logger.recordOutput(getName() + "/demand", demand);
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
        Logger.recordOutput(getName() + "/angle", angle);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
