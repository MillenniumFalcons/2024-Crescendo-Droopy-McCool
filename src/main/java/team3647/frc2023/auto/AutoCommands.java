package team3647.frc2023.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.SwerveDrive;

public class AutoCommands {
    private final SwerveDrive swerve;

    private final String s1_to_n1 = "s1 to n1";
    private final String s1_to_n1_to_f1 = "s1 to n1 to f1";
    private final String s2_to_n2 = "s2 to n2";
    private final String n2_to_n1 = "n2 to n1";
    private final String n2_to_n3 = "n2 to n3";
    private final String s3_to_n3 = "s3 to n3";
    private final String n3_to_f5 = "n3 to f5";
    private final String n1_to_n2 = "n1 to n2";
    private final String n3_to_n2 = "n3 to n2";
    private final String f1_to_n2 = "f1 to n2";

    public final AutonomousMode blueFive_S1N1F1N2N3;

    public final AutonomousMode blueFour_S1N1N2N3;

    // public final AutonomousMode Test;

    public AutoCommands(SwerveDrive swerve) {
        this.swerve = swerve;

        this.blueFive_S1N1F1N2N3 =
                new AutonomousMode(five_S1N1F1N2N3(Alliance.Blue), getInitial(s1_to_n1_to_f1));

        this.blueFour_S1N1N2N3 =
                new AutonomousMode(four_S1N1N2N3(Alliance.Blue), getInitial(s1_to_n1));
        // Test = new AutonomousMode(drive(), Trajectories.Blue.Test.kStar
    }

    public void registerCommands() {}

    public Command five_S1N1F1N2N3(Alliance color) {
        return Commands.sequence(
                followChoreoPath(s1_to_n1_to_f1, color),
                followChoreoPath(f1_to_n2, color),
                followChoreoPath(n2_to_n3, color));
    }

    public Command four_S1N1N2N3(Alliance color) {
        return Commands.sequence(
                followChoreoPath(s1_to_n1, color),
                Commands.waitSeconds(1),
                followChoreoPath(n1_to_n2, color),
                Commands.waitSeconds(1),
                followChoreoPath(n2_to_n3, color));
    }

    public Pose2d getInitial(String path) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        return traj.getInitialPose();
    }

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

    public Command followChoreoPath(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        return Choreo.choreoSwerveCommand(
                traj,
                swerve::getOdoPose,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                (ChassisSpeeds speeds) ->
                        swerve.drive(
                                new Translation2d(
                                        speeds.vxMetersPerSecond / 4.279,
                                        speeds.vyMetersPerSecond / 4.279),
                                speeds.omegaRadiansPerSecond / 13.449,
                                false,
                                true),
                () -> mirror,
                swerve);
    }
}
