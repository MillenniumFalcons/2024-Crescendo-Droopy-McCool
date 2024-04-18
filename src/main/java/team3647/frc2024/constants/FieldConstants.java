// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2024.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class FieldConstants {
    public static final Rotation2d kOneEighty = Rotation2d.fromDegrees(180);
    public static final Rotation2d kZero = new Rotation2d();
    public static final Rotation2d kNinety = Rotation2d.fromDegrees(90);

    public static final double kFieldLength = Units.inchesToMeters(651.25);
    public static final double kFieldWidth = Units.inchesToMeters(315.5);

    public static final Pose2d kOrigin = new Pose2d();

    public static final double kSpeakerHeight = Units.inchesToMeters(100);

    public static final Pose2d kBlueSpeaker = new Pose2d(0, 5.55, new Rotation2d());

    public static final Pose2d kRedSpeaker = new Pose2d(16.5, 6.5, new Rotation2d());

    public static final Pose3d kSpeaker = null;

    public static final double kAmpHeight = Units.inchesToMeters(35);

    public static final Pose2d kBlueFeed = new Pose2d(0, kFieldWidth - 1, new Rotation2d());

    public static final Pose2d kBlueAmp = new Pose2d(1.84, 7.82, new Rotation2d());

    public static final Pose2d kRedAmp = new Pose2d(14.7, 7.82, new Rotation2d());

    public static final Pose3d kAmp = null;

    public static final Pose2d[] kBlueAutoNotePoses = {
        new Pose2d(2.912, 6.951, kZero), // n1
        new Pose2d(2.912, 5.508, kZero), // n2
        new Pose2d(2.912, 4.046, kZero), // n3
        new Pose2d(8.274, 6.951, kZero), // f1
        new Pose2d(8.274, 5.723, kZero), // f2
        new Pose2d(8.274, 4.065, kZero), // f3
        new Pose2d(8.274, 2.369, kZero), // f4
        new Pose2d(8.274, 0.712, kZero) // f5
    };

    public static final Pose2d kBlueAmpAlign =
            new Pose2d(
                    1.82,
                    7.72,
                    Rotation2d.fromDegrees(90)); // rotation is heading, not holonomic rotation

    public static Translation2d flipTranslation(Translation2d translation) {
        return new Translation2d(
                FieldConstants.kFieldLength - translation.getX(), translation.getY());
    }

    public static Pose2d flipBluePose(Pose2d pose) {
        return new Pose2d(
                flipTranslation(pose.getTranslation()),
                new Rotation2d(pose.getRotation().getCos() * -1, pose.getRotation().getSin()));
    }

    public static Transform2d flipBlueTransform(Transform2d transform) {
        return new Transform2d(kOrigin, flipBluePose(kOrigin.transformBy(transform)));
    }

    static {
        // kSpeaker = DriverStation.getAlliance().get() == Alliance.Blue ? kBlueSpeaker :
        // kRedSpeaker;
        // kAmp = DriverStation.getAlliance().get() == Alliance.Blue ? kBlueAmp : kRedAmp;
    }

    private FieldConstants() {}
}
