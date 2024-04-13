package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;
import team3647.lib.TalonFXSubsystem;

public class ClimbLeft extends TalonFXSubsystem {

    private final double minLength;
    private double maxLength;

    BooleanSupplier underStage;

    private final double maxKG;

    public ClimbLeft(
            TalonFX master,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double minLength,
            double maxLength,
            double nominalVoltage,
            double maxKG,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.minLength = minLength;
        this.maxLength = maxLength;
        this.maxKG = maxKG;
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
    }

    public void setPosition(double position) {
        super.setPositionMotionMagic(position, 0);
    }

    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    public double getLength() {
        return super.getPosition();
    }

    public boolean LengthReached(double targetLength, double threshold) {
        return Math.abs(getLength() - targetLength) < threshold;
    }

    @Override
    public String getName() {
        return "Climb Left";
    }
}
