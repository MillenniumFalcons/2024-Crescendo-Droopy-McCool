package team3647.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2024.constants.ClimbConstants;
import team3647.frc2024.subsystems.Climb;

public class ClimbCommands {
    private final Set<Subsystem> requirements;

    public Command goUp() {
        return Commands.run(() -> climb.openLoop(1), climb);
    }

    public Command goDown() {
        return Commands.run(() -> climb.openLoop(-1), climb);
    }

    public Command kill() {
        return Commands.run(() -> climb.openLoop(0), climb);
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = ClimbConstants.kInitialLength;

            @Override
            public void initialize() {
                degreeAtStart = climb.getLength();
            }

            @Override
            public void execute() {
                climb.setPosition(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    public ClimbCommands(Climb climb) {
        this.climb = climb;
        this.requirements = Set.of(climb);
    }

    private final Climb climb;
}
