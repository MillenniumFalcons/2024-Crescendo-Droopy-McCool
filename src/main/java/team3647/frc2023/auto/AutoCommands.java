package team3647.frc2023.auto;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.SwerveDrive;

public class AutoCommands {
    private final SwerveDrive swerve;

    // public final AutonomousMode Test;

    public AutoCommands(SwerveDrive swerve) {
        this.swerve = swerve;
        // Test = new AutonomousMode(drive(), Trajectories.Blue.Test.kStar
    }

    public void registerCommands() {}

    public Pose2d flipForPP(Pose2d pose) {
        return new Pose2d(
                new Translation2d(pose.getX(), FieldConstants.kFieldWidth - pose.getY()),
                pose.getRotation());
    }

    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathWithEvents(
                new FollowPathHolonomic(
                        path,
                        swerve::getOdoPose,
                        swerve::getChassisSpeeds,
                        swerve::drive,
                        new HolonomicPathFollowerConfig(
                                AutoConstants.kTranslationConstants,
                                AutoConstants.kRotationConstants,
                                5,
                                SwerveDriveConstants.kTrackWidth / 2.0 * Math.sqrt(2.0),
                                new ReplanningConfig()),
                        swerve),
                path,
                swerve::getOdoPose);
    }
}
