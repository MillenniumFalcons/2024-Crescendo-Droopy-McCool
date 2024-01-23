package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2023.subsystems.Kicker;

public class KickerCommands {

    public Command kick() {
        return Commands.run(() -> kicker.openLoop(0.5), kicker);
    }

    public Command unkick() {
        return Commands.run(() -> kicker.openLoop(-0.5), kicker);
    }

    public Command kill() {
        return Commands.run(() -> kicker.openLoop(0), kicker);
    }

    private final Set<Subsystem> requirements;

    public KickerCommands(Kicker kicker) {
        this.kicker = kicker;
        this.requirements = Set.of(kicker);
    }

    private final Kicker kicker;
}
