package team3647.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2024.constants.ClimbConstants;
import team3647.frc2024.subsystems.ClimbLeft;
import team3647.frc2024.subsystems.ClimbRight;

public class ClimbCommands {
    private final Set<Subsystem> requirements;

    public Command goUp() {
        return Commands.run(
                () -> {
                    // climbLeft.openLoop(1);
                    // climbRight.openLoop(1);
                },
                climbLeft,
                climbRight);
    }

    public Command goDown() {
        return Commands.run(
                () -> {
                    // climbLeft.openLoop(-1);
                    // climbRight.openLoop(-1);
                },
                climbLeft,
                climbRight);
    }

    public Command kill() {
        return Commands.runOnce(
                () -> {
                    climbLeft.openLoop(0);
                    climbRight.openLoop(0);
                },
                climbLeft,
                climbRight);
    }

    public Command holdRightPositionAtCall() {
        return new Command() {
            double degreeAtStart = ClimbConstants.kInitialLength;

            @Override
            public void initialize() {
                degreeAtStart = climbRight.getLength();
            }

            @Override
            public void execute() {
                climbRight.setPosition(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(climbRight);
            }
        };
    }

    public Command holdLeftPositionAtCall() {
        return new Command() {
            double degreeAtStart = ClimbConstants.kInitialLength;

            @Override
            public void initialize() {
                degreeAtStart = climbLeft.getLength();
            }

            @Override
            public void execute() {
                climbLeft.setPosition(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(climbLeft);
            }
        };
    }

    public ClimbCommands(ClimbLeft climbLeft, ClimbRight climbRight) {
        this.climbLeft = climbLeft;
        this.climbRight = climbRight;
        this.requirements = Set.of(climbLeft, climbRight);
    }

    private final ClimbLeft climbLeft;
    private final ClimbRight climbRight;
}
