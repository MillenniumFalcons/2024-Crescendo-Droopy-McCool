package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class Intake extends TalonFXSubsystem {
    public Intake(
            TalonFX master,
            TalonFX follower,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        super.addFollower(follower, false);
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
        Logger.recordOutput(getName() + "/demand", demand);
    }

    @Override
    public String getName() {
        return "Intake";
    }
}
