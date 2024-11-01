package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class ShooterRight extends TalonFXSubsystem {

    private final SimpleMotorFeedforward ff;

    public ShooterRight(
            TalonFX master,
            TalonFX follower,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.addFollower(follower, true);
        this.ff = ff;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    public boolean velocityGreater(double setpoint) {
        return super.getVelocity() > setpoint;
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
    }

    public void setTorque(double torque) {
        super.setTorque(torque);
    }

    public void setVelocity(double velocity) {
        super.setVelocityVoltage(velocity, 0);
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
