package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2023.subsystems.Intake;

public class IntakeCommands {

    public Command intake() {
        return Commands.run(() -> intake.openLoop(0.5));
    }

    public Command kill() {
        return Commands.run(() -> intake.openLoop(0));
    }

    private final Set<Subsystem> requirements;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
        this.requirements = Set.of(intake);
    }

    private final Intake intake;
}
