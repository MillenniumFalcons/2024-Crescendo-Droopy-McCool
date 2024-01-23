package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2023.subsystems.Intake;

public class IntakeCommands {

    public Command intake() {
        return Commands.run(() -> intake.openLoop(0.3), intake);
    }

    public Command outtake() {
        return Commands.run(() -> intake.openLoop(-0.3), intake);
    }

    public Command kill() {
        return Commands.run(() -> intake.openLoop(0), intake);
    }

    private final Set<Subsystem> requirements;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
        this.requirements = Set.of(intake);
    }

    private final Intake intake;
}
