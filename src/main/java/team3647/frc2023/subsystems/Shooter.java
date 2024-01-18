package team3647.frc2023.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class Shooter extends TalonFXSubsystem {

    private final SimpleMotorFeedforward ff;

    public Shooter(
            TalonFX master,
            TalonFX follower,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        super.addFollower(follower, false);
        this.ff = ff;
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
        Logger.recordOutput(getName() + "/openLoop", demand);
    }

    public void setVelocity(double velocity) {
        super.setVelocity(velocity, ff.calculate(velocity));
        Logger.recordOutput(getName() + "/velocity", velocity);
    }

    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
        Logger.recordOutput(getName() + "/voltage", voltage);
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}
