package team3647.frc2023.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import team3647.lib.GeomUtil;

public class TargetingUtil {

    final Pose2d speakerPose;
    final double speakerHeight;
    final Supplier<Pose2d> drivePose;
    final Supplier<ChassisSpeeds> fieldRelativeSpeeds;
    final Transform3d robotToShooter;
    double kDt = 0.02;

    public TargetingUtil(
            Pose2d speakerPose,
            double speakerHeight,
            Supplier<Pose2d> drivePose,
            Supplier<ChassisSpeeds> fieldRelativeSpeeds,
            Transform3d robotToShooter) {
        this.speakerPose = speakerPose;
        this.speakerHeight = speakerHeight;
        this.drivePose = drivePose;
        this.fieldRelativeSpeeds = fieldRelativeSpeeds;
        this.robotToShooter = robotToShooter;
    }

    public double angleToSpeaker() { // if facing to the left, returns positive
        var currentPose = drivePose.get();
        var toSpeaker =
                VecBuilder.fill(
                        speakerPose.getX() - currentPose.getX(),
                        speakerPose.getY() - currentPose.getY());
        var driveOrientation =
                VecBuilder.fill(
                        Math.cos(currentPose.getRotation().getRadians()),
                        Math.sin(currentPose.getRotation().getRadians()));
        double angle = Math.acos(toSpeaker.dot(driveOrientation) / toSpeaker.norm());
        double fieldAngle =
                Math.acos(toSpeaker.dot(VecBuilder.fill(-1, 0)) / toSpeaker.norm())
                        * Math.signum(currentPose.getY() - speakerPose.getY());
        boolean shouldBeNegative =
                Math.sin(currentPose.getRotation().getRadians() - fieldAngle) > 0;
        int negative = shouldBeNegative ? -1 : 1;
        // SmartDashboard.putNumber("angle", Units.radiansToDegrees(angle) * negative);
        return angle * negative;
    }

    public Pose2d compensatedPose() {
        var twist =
                new Twist2d(
                        fieldRelativeSpeeds.get().vxMetersPerSecond * kDt,
                        fieldRelativeSpeeds.get().vyMetersPerSecond * kDt,
                        fieldRelativeSpeeds.get().omegaRadiansPerSecond * kDt);
        var newPose = drivePose.get().exp(twist);
        return newPose;
    }

    public double angleToSpeakerCompensated() { // if facing to the left, returns positive
        final var currentPose = compensatedPose();
        final var toSpeaker =
                VecBuilder.fill(
                        speakerPose.getX() - currentPose.getX(),
                        speakerPose.getY() - currentPose.getY());
        final var driveOrientation =
                VecBuilder.fill(
                        Math.cos(currentPose.getRotation().getRadians()),
                        Math.sin(currentPose.getRotation().getRadians()));
        final double angle = Math.acos(toSpeaker.dot(driveOrientation) / toSpeaker.norm());
        final double fieldAngle =
                Math.acos(toSpeaker.dot(VecBuilder.fill(-1, 0)) / toSpeaker.norm())
                        * Math.signum(currentPose.getY() - speakerPose.getY());
        final boolean shouldBeNegative =
                Math.sin(currentPose.getRotation().getRadians() - fieldAngle) < 0;
        final int negative = shouldBeNegative ? -1 : 1;
        // SmartDashboard.putNumber("angle", Units.radiansToDegrees(angle) * negative);
        return angle * negative;
    }

    public double fieldAngleToSpeakerOnTheMove() {
        var currentPose = compensatedPose();
        double angle = currentPose.getRotation().getRadians() + angleToSpeakerCompensated();
        SmartDashboard.putNumber("compensated angle", angleToSpeakerCompensated());
        while (Math.abs(angle) > Math.PI) {
            angle -= 2 * Math.PI * Math.signum(angle);
        }
        SmartDashboard.putNumber("current pose angle", currentPose.getRotation().getRadians());
        SmartDashboard.putNumber("compensated speaker angle", angleToSpeakerCompensated());
        var newAngle =
                Math.atan(
                        (exitVelocity()
                                                * Math.cos(getPivotAngleByDistanceCompensated())
                                                * Math.sin(angle)
                                        - fieldRelativeSpeeds.get().vyMetersPerSecond)
                                / (exitVelocity()
                                                * Math.cos(getPivotAngleByDistanceCompensated())
                                                * Math.cos(angle)
                                        - fieldRelativeSpeeds.get().vxMetersPerSecond));
        boolean shouldAddPi = Math.cos(angle) < 0;
        double pi = shouldAddPi ? Math.PI : 0;
        boolean shouldSubtract = Math.sin(angle) < 0;
        pi = shouldSubtract ? -pi : pi;
        newAngle = newAngle + pi;
        SmartDashboard.putNumber("new angle", newAngle);
        SmartDashboard.putNumber("angle", angle);
        return newAngle;
    }

    public double angleToSpeakerOnTheMove() {
        var currentPose = compensatedPose();
        var rot = currentPose.getRotation().getRadians();
        var speak = fieldAngleToSpeakerOnTheMove();
        if (Math.signum(rot * speak) < 0) {
            if (rot < 0) {
                rot += 2 * Math.PI;
            } else {
                rot -= 2 * Math.PI;
            }
        }
        var newAngleToSpeaker = rot - fieldAngleToSpeakerOnTheMove();
        SmartDashboard.putNumber("angle output", newAngleToSpeaker);
        // SmartDashboard.putNumber("angle", newAngleToSpeaker);

        return newAngleToSpeaker;
    }

    public double exitVelocity() {
        return 12;
    }

    public double getPivotAngleByDistance() {
        var currentPose = drivePose.get();
        var pose3D =
                new Pose3d(
                        currentPose.getX(),
                        currentPose.getY(),
                        0,
                        new Rotation3d(0, 0, currentPose.getRotation().getRadians()));
        var shooterPose = pose3D.transformBy(robotToShooter);
        var shooter2D = shooterPose.toPose2d();
        double shooterDistance = GeomUtil.distance(speakerPose.minus(shooter2D));
        return Math.atan(speakerHeight / shooterDistance);
    }

    public double getPivotAngleByDistanceCompensated() {
        var currentPose = compensatedPose();
        var pose3D =
                new Pose3d(
                        currentPose.getX(),
                        currentPose.getY(),
                        0,
                        new Rotation3d(0, 0, currentPose.getRotation().getRadians()));
        var shooterPose = pose3D.transformBy(robotToShooter);
        var shooter2D = shooterPose.toPose2d();
        double shooterDistance = GeomUtil.distance(speakerPose.minus(shooter2D));
        return Math.atan(speakerHeight / shooterDistance);
    }

    public double getPivotAngleByDistanceOnTheMove() {
        var currentPose = compensatedPose();
        double angle = currentPose.getRotation().getRadians() - angleToSpeakerCompensated();
        double pivotAngle = getPivotAngleByDistanceCompensated();
        double newPivotAngle =
                Math.atan(
                        (exitVelocity()
                                        * Math.sin(pivotAngle)
                                        * Math.cos(fieldAngleToSpeakerOnTheMove()))
                                / (exitVelocity() * Math.cos(pivotAngle) * Math.cos(angle)
                                        + fieldRelativeSpeeds.get().vxMetersPerSecond));
        return newPivotAngle;
    }
}
