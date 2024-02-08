package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class ShooterRight extends TalonFXSubsystem {

    private final SimpleMotorFeedforward ff;

    public ShooterRight(
            TalonFX master,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.ff = ff;
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
    }

    public void setVelocity(double velocity) {
        super.setVelocity(velocity, ff.calculate(velocity));
    }

    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    public boolean velocityReached(double setpoint, double threshold) {
        return Math.abs(super.getVelocity() - setpoint) < threshold;
    }

    @Override
    public String getName() {
        return "Shooter Right";
    }
}