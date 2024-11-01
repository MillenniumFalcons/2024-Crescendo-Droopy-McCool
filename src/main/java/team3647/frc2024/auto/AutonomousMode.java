package team3647.frc2024.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import team3647.lib.Elastic;
import team3647.lib.Elastic.ElasticNotification;
import team3647.lib.Elastic.ElasticNotification.NotificationLevel;

public class AutonomousMode {
    public AutonomousMode(Command autoCommand, Pose2d ppinitial, String name) {
        this.autoCommand = autoCommand;
        this.ppInitial = ppinitial;
        this.name = name;
    }

    public AutonomousMode(Command autoCommand, Pose2d ppinitial) {
        this(autoCommand, ppinitial, "no name");
        DriverStation.reportWarning("you have an unamed auto!", false);
        Elastic.sendAlert(
            new ElasticNotification(
                NotificationLevel.WARNING, "Unamed Auto!",
                "One of your autos is Unamed! Object ID: " + this.toString()));
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
