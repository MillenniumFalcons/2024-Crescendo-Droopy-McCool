package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
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
    }

    @Override
    public String getName() {
        return "Kicker";
    }
}
