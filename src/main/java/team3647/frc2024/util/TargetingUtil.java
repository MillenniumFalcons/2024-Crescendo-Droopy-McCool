package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import team3647.frc2024.constants.FieldConstants;
import team3647.frc2024.constants.PivotConstants;

public class TargetingUtil {

    private final Pose3d speakerPose;
    private final Pose3d ampPose;
    private final Supplier<Pose2d> drivePose;
    private final Supplier<ChassisSpeeds> robotRelativeSpeeds;
    private final Transform2d robotToShooter;
    private final double shootSpeed = 30;
    private double offset = 0;
    double kDt = 0.1;

    private final InterpolatingDoubleTreeMap speakerMap = PivotConstants.kMasterSpeakerMap;

    private final InterpolatingDoubleTreeMap ampMap = PivotConstants.kMasterAmpMap;

    public TargetingUtil(
            Pose3d speakerPose,
            Pose3d ampPose,
            Supplier<Pose2d> drivePose,
            Supplier<ChassisSpeeds> robotRelativeSpeeds,
            Transform2d robotToShooter) {
        this.speakerPose = speakerPose;
        this.ampPose = ampPose;
        this.drivePose = drivePose;
        this.robotRelativeSpeeds = robotRelativeSpeeds;
        this.robotToShooter = robotToShooter;
    }

    public Command offsetUp() {
        return Commands.runOnce(() -> offset += 1);
    }

    public Command offsetDown() {
        return Commands.runOnce(() -> offset -= 1);
    }

    public boolean underStage() {
        return ((compensatedPose().getX() > 3.5 && compensatedPose().getX() < 6.2)
                        || (compensatedPose().getX() > 10.2 && compensatedPose().getX() < 12.9))
                && ((Math.abs(compensatedPose().getY() - 4)
                                        < ((compensatedPose().getX() - 2.9) * 1 / 1.73)
                                && compensatedPose().getX() < 6.2)
                        || (Math.abs(compensatedPose().getY() - 4)
                                        < ((13.6 - compensatedPose().getX()) * 1 / 1.73)
                                && compensatedPose().getX() > 10.2));
    }

    // returns an object storing a pair of doubles, swerve angle change and pivot angle
    public AimingParameters shootAtPose(Pose3d pose) {
        final var currentPose = compensatedPose();
        var shooterPose = currentPose.transformBy(robotToShooter);
        double shooterDistance = pose.toPose2d().minus(shooterPose).getTranslation().getNorm();
        double pivotAngle = Math.toRadians(speakerMap.get(shooterDistance) + offset);
        if (pose.getZ() == FieldConstants.kAmpHeight) {
            pivotAngle =
                    Math.toRadians(ampMap.get(shooterDistance) + offset); // pivot angle stationary
        }
        final var toPose = currentPose.minus(pose.toPose2d()).getTranslation();
        double angle =
                Math.acos(toPose.getX() / toPose.getNorm())
                        * Math.signum(toPose.getY())
                        * Math.signum(toPose.getX()); // field angle to pose stationary
        var newAngle =
                Math.atan(
                        (exitVelocity() * Math.cos(pivotAngle) * Math.sin(angle)
                                        + robotRelativeSpeeds.get().vyMetersPerSecond * 2)
                                / (exitVelocity() * Math.cos(pivotAngle) * Math.cos(angle)
                                        + robotRelativeSpeeds.get().vxMetersPerSecond * 2));
        boolean shouldAddPi = Math.cos(newAngle) < 0;
        double pi = shouldAddPi ? Math.PI : 0;
        boolean shouldSubtract = Math.sin(newAngle) < 0;
        pi = shouldSubtract ? -pi : pi;
        newAngle = newAngle + pi; // field angle to pose
        double robotAngleToPose = currentPose.getRotation().getRadians() - newAngle;
        double angleOnTheMove = newAngle;
        if (angleOnTheMove < 0) {
            angleOnTheMove += Math.PI;
        } else {
            angleOnTheMove -= Math.PI;
        }
        double angleStationary = angle;
        if (angleStationary < 0) {
            angleStationary += Math.PI;
        } else {
            angleStationary -= Math.PI;
        }
        double newPivotAngle =
                Math.atan(
                        (exitVelocity() * Math.sin(pivotAngle) * Math.cos(angleStationary))
                                / (exitVelocity() * Math.cos(pivotAngle) * Math.cos(angleOnTheMove)
                                        - robotRelativeSpeeds.get().vxMetersPerSecond * 2));
        return new AimingParameters(robotAngleToPose, newPivotAngle, shootSpeed);
    }

    public AimingParameters shootAtPose(Pose3d pose, double shootSpeed) {
        return shootAtPose(pose).withShootSpeed(shootSpeed);
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtSpeaker() {
        return shootAtPose(speakerPose);
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtAmp() {
        return shootAtPose(ampPose, 5.5);
    }

    public double distance() {
        var currentPose = compensatedPose();
        var shooterPose = currentPose.transformBy(robotToShooter);
        var shooterDistance = speakerPose.toPose2d().minus(shooterPose);
        return shooterDistance.getTranslation().getNorm();
    }

    public double getPivotAngleByPose(Pose2d botPose) {
        var currentPose = botPose;
        var shooterPose = currentPose.transformBy(robotToShooter);
        var pose = speakerPose;
        double shooterDistance = pose.toPose2d().minus(shooterPose).getTranslation().getNorm();
        double pivotAngle = Math.toRadians(speakerMap.get(shooterDistance) + offset);
        return pivotAngle;
    }

    // returns latency compensated pose;
    public Pose2d compensatedPose() {
        var speeds = robotRelativeSpeeds.get();
        var twist =
                new Twist2d(
                        speeds.vxMetersPerSecond * kDt,
                        speeds.vyMetersPerSecond * kDt,
                        speeds.omegaRadiansPerSecond * kDt);
        var newPose = drivePose.get().exp(twist);
        return newPose;
    }

    // returns shot exit velocity
    public double exitVelocity() {
        return shootSpeed;
    }

    // for cleaner packaging
    public class AimingParameters {
        public double rotation;
        public double pivot;
        public double shootSpeed;

        public AimingParameters(double rotation, double pivot, double shootSpeed) {
            this.rotation = rotation;
            this.pivot = pivot;
            this.shootSpeed = shootSpeed;
        }

        public AimingParameters withShootSpeed(double speed) {
            this.shootSpeed = speed;
            return this;
        }

        public AimingParameters withRotation(double rotation) {
            this.rotation = rotation;
            return this;
        }

        public AimingParameters withPivot(double pivot) {
            this.pivot = pivot;
            return this;
        }
    }
}
