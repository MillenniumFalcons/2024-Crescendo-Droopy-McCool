package team3647.frc2023.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team3647.frc2023.constants.FieldConstants;

public final class Trajectories {

    public static final PathConstraints defaultConstraints = new PathConstraints(4, 3, 4, 3);

    public static Pose2d flipForAlliance(Pose2d pose, Alliance color) {
        if (color == Alliance.Red) {
            return new Pose2d(
                    FieldConstants.kFieldLength - pose.getX(),
                    pose.getY(),
                    new Rotation2d(Math.PI - pose.getRotation().getRadians()));
        } else {
            return pose;
        }
    }

    public static Pose2d[] flipForAlliance(Pose2d[] poses, Alliance color) {
        if (color == Alliance.Blue) {
            return poses;
        }
        Pose2d[] newPoses = poses;
        for (int i = 0; i < poses.length; i++) {
            Pose2d pose = poses[i];
            if (color == Alliance.Red) {
                newPoses[i] = flipForAlliance(pose, color);
            }
        }
        return newPoses;
    }

    public static GoalEndState flipForAlliance(GoalEndState goal, Alliance color) {
        if (color == Alliance.Red) {
            return new GoalEndState(
                    goal.getVelocity(), new Rotation2d(Math.PI - goal.getRotation().getRadians()));
        } else {
            return goal;
        }
    }

    public static RotationTarget flipForAlliance(RotationTarget target, Alliance color) {
        if (color == Alliance.Red) {
            return new RotationTarget(
                    target.getPosition(),
                    new Rotation2d(Math.PI - target.getTarget().getRadians()));
        } else {
            return target;
        }
    }

    public final class Blue {}
}
