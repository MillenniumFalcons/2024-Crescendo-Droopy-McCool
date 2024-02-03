package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.util.Units;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {
    private final double minDegree;
    private final double maxDegree;
    private final double kG;
    private final TimeOfFlight tof;

    public Wrist(
            TalonFX master,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double minDegree,
            double maxDegree,
            double nominalVoltage,
            double kG,
            double kDt,
            TimeOfFlight tof) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        this.minDegree = minDegree;
        this.maxDegree = maxDegree;
        this.kG = kG;
        this.tof = tof;
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    public boolean hasPiece() {
        return this.tof.getRange() < 300;
    }

    public double getAngle() {
        return super.getPosition();
    }

    public void setAngle(double angle) {
        var ffVolts = kG * Math.cos(Units.degreesToRadians(angle));
        this.setPositionMotionMagic(angle, ffVolts);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
