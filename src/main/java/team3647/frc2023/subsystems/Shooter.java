package team3647.frc2023.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class Shooter extends TalonFXSubsystem {

    public Shooter(
            TalonFX master,
            TalonFX follower,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        super.addFollower(follower, true);
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
        Logger.recordOutput(getName() + "/output", demand);
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}
