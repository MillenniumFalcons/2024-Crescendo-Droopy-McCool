package team3647.frc2024.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousMode {
    public AutonomousMode(Command autoCommand, Pose2d ppinitial, String name) {
        this.autoCommand = autoCommand;
        this.ppInitial = ppinitial;
        this.name = name;
    }

    public AutonomousMode(Command autoCommand, Pose2d ppinitial) {
        this(autoCommand, ppinitial, "no name");
    }

    private final Command autoCommand;

    public Command getAutoCommand() {
        return autoCommand;
    }

    private final Pose2d ppInitial;

    public Pose2d getPathplannerPose2d() {
        return ppInitial;
    }

    private final String name;

    public String getName() {
        return name;
    }
}
