package team3647.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2024.subsystems.Intake;

public class IntakeCommands {

    public Command intake() {
        return Commands.run(() -> intake.openLoop(1), intake);
    }

    public Command outtake() {
        return Commands.run(() -> intake.openLoop(-0.1), intake);
    }

    public Command spitOut() {
        return Commands.run(() -> intake.openLoop(-0.8), intake);
    }

    public Command oscillate() {
        return Commands.run(() -> intake.openLoop(0.1))
                .withTimeout(0.2)
                .andThen(Commands.run(() -> intake.openLoop(-0.1)))
                .withTimeout(0.2)
                .repeatedly();
    }

    public Command kill() {
        return Commands.runOnce(() -> intake.openLoop(0), intake);
    }

    public Command intakespeedup(){
        return Commands.runOnce(() -> intakeSpeed +=0.5);
    }

    public Command intakespeeddown(){
        return Commands.runOnce(() -> intakeSpeed -=0.5);
    }


    private final Set<Subsystem> requirements;
    private double intakeSpeed = 0.8;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
        this.requirements = Set.of(intake);
    }

    private final Intake intake;
}
