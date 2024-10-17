package team3647.frc2024.util;

import java.nio.file.attribute.PosixFileAttributes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    /**
     * 
     * do NOT use this if its on the wrong side
     * 
     */
    public static Pose2d flipForPP(Pose2d pose, Alliance color){
        if(color == Alliance.Red){
            return flipForPP(pose, pose.getX() < 8.29);
        }
        return flipForPP(pose, pose.getX() > 8.29);
    }

    public static Pose2d makeRedSide(Pose2d pose){
        if (pose.getX() <8.29){
            return pose;
        }
        return flipForPP(pose);
    }

    public static Pose2d makeBlueSide(Pose2d pose){
        if(pose.getX() > 8.29){
            return pose;
        }
        return flipForPP(pose);
    }
}
