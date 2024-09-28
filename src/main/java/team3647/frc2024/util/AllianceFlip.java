package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import team3647.frc2024.constants.FieldConstants;

public class AllianceFlip {
    public static Pose2d flipForPP(Pose2d pose, boolean shouldFlip) {
        if (!shouldFlip) {
            return pose;
        }
        return new Pose2d(
                new Translation2d(FieldConstants.kFieldLength - pose.getX(), pose.getY()),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    }

    public static Pose2d flipForPP(Pose2d pose) {
        return new Pose2d(
                new Translation2d(FieldConstants.kFieldLength - pose.getX(), pose.getY()),
                new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    }
}
