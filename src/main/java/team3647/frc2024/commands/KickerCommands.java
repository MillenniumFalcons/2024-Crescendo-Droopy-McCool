package team3647.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2024.subsystems.Kicker;

public class KickerCommands {

    public Command kick() {
        return Commands.run(() -> kicker.openLoop(1), kicker);
    }

    public Command fastKick() {
        return Commands.run(() -> kicker.openLoop(1), kicker);
    }

    public Command slowFeed() {
        return Commands.run(() -> kicker.openLoop(0.2), kicker);
    }

    public Command unkick() {
        return Commands.run(() -> kicker.openLoop(-0.15), kicker);
    }

    public Command oscillate() {
        return Commands.run(() -> kicker.openLoop(0.1))
                .withTimeout(0.2)
                .andThen(Commands.run(() -> kicker.openLoop(-0.1)))
                .withTimeout(0.2)
                .repeatedly();
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
