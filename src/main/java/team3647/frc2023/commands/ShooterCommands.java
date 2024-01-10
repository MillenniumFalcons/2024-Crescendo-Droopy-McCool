package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Shooter;

public class ShooterCommands {
    public final Shooter shooter;

    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
    }

    public Command shoot(DoubleSupplier bill) {
        return Commands.run(() -> shooter.openLoop(bill.getAsDouble()), shooter);
    }

    public Command kill() {
        return Commands.run(() -> shooter.openLoop(0));
    }
}
