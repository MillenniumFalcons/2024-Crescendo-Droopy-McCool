package team3647.frc2023.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class Kicker extends TalonFXSubsystem {
    public Kicker(
            TalonFX master,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
        Logger.recordOutput(getName() + "/demand", demand);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Kicker";
    }
}
