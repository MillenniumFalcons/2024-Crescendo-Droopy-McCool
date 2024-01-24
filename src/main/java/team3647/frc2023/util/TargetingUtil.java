package team3647.frc2023.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import team3647.lib.GeomUtil;

public class TargetingUtil {

    private final Pose3d speakerPose;
    private final Supplier<Pose2d> drivePose;
    private final Supplier<ChassisSpeeds> fieldRelativeSpeeds;
    private final Transform3d robotToShooter;
    double kDt = 0.02;

    public TargetingUtil(
            Pose3d speakerPose,
            Supplier<Pose2d> drivePose,
            Supplier<ChassisSpeeds> fieldRelativeSpeeds,
            Transform3d robotToShooter) {
        this.speakerPose = speakerPose;
        this.drivePose = drivePose;
        this.fieldRelativeSpeeds = fieldRelativeSpeeds;
        this.robotToShooter = robotToShooter;
    }

    // returns an object storing a pair of doubles, swerve angle change and pivot angle
    public AimingParameters shootAtPose(Pose3d pose) {
        return new AimingParameters(robotAngleToPose(pose), getPivotAngle(pose));
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtSpeaker() {
        return shootAtPose(speakerPose);
    }

    // returns the angle between the vector pointing straight back and the pose
    public double fieldAngleToPose(Pose3d pose) {
        final var currentPose = compensatedPose();
        final var toPose =
                VecBuilder.fill(pose.getX() - currentPose.getX(), pose.getY() - currentPose.getY());
        double angle =
                Math.acos(toPose.dot(VecBuilder.fill(-1, 0)) / toPose.norm())
                        * Math.signum(currentPose.getY() - pose.getY());
        var newAngle =
                Math.atan(
                        (exitVelocity() * Math.cos(getPivotAngleStationary(pose)) * Math.sin(angle)
                                        - fieldRelativeSpeeds.get().vyMetersPerSecond)
                                / (exitVelocity()
                                                * Math.cos(getPivotAngleStationary(pose))
                                                * Math.cos(angle)
                                        - fieldRelativeSpeeds.get().vxMetersPerSecond));
        boolean shouldAddPi = Math.cos(newAngle) < 0;
        double pi = shouldAddPi ? Math.PI : 0;
        boolean shouldSubtract = Math.sin(newAngle) < 0;
        pi = shouldSubtract ? -pi : pi;
        newAngle = newAngle + pi;
        return newAngle;
    }

    // returns the angle between the vector pointing straight back and the pose without accounting
    // for velocity
    public double fieldAngleToPoseStationary(Pose3d pose) {
        final var currentPose = compensatedPose();
        final var toPose =
                VecBuilder.fill(pose.getX() - currentPose.getX(), pose.getY() - currentPose.getY());
        double angle =
                Math.acos(toPose.dot(VecBuilder.fill(-1, 0)) / toPose.norm())
                        * Math.signum(currentPose.getY() - pose.getY());
        return angle;
    }

    // return the angle between the robots back heading (since shooter is at back) and the pose
    public double robotAngleToPose(Pose3d pose) {
        final var currentPose = compensatedPose();
        var rot = currentPose.getRotation().getRadians();
        final var fieldAngle = fieldAngleToPose(pose);
        if (Math.signum(rot * fieldAngle) < 0) {
            if (rot < 0) {
                rot += 2 * Math.PI;
            } else {
                rot -= 2 * Math.PI;
            }
        }
        var newAngle = rot - fieldAngle;
        if (newAngle > 0) {
            newAngle -= Math.PI;
        } else {
            newAngle += Math.PI;
        }

        return newAngle;
    }

    // returns the pivot angle
    public double getPivotAngle(Pose3d pose) {
        double angleOnTheMove = fieldAngleToPose(pose);
        if (angleOnTheMove < 0) {
            angleOnTheMove += Math.PI;
        } else {
            angleOnTheMove -= Math.PI;
        }
        double angle = fieldAngleToPoseStationary(pose);
        if (angle < 0) {
            angle += Math.PI;
        } else {
            angle -= Math.PI;
        }
        var currentPose = compensatedPose();
        var pose3D =
                new Pose3d(
                        currentPose.getX(),
                        currentPose.getY(),
                        0,
                        new Rotation3d(0, 0, currentPose.getRotation().getRadians()));
        var shooterPose = pose3D.transformBy(robotToShooter);
        var shooter2D = shooterPose.toPose2d();
        double shooterDistance = GeomUtil.distance(pose.toPose2d().minus(shooter2D));
        double pivotAngle =
                Math.atan((pose.getZ() - shooterPose.getZ()) / shooterDistance)
                        + shooterDistance * Math.PI / 180 * 1;
        double newPivotAngle =
                Math.atan(
                        (exitVelocity() * Math.sin(pivotAngle) * Math.cos(angle))
                                / (exitVelocity() * Math.cos(pivotAngle) * Math.cos(angleOnTheMove)
                                        + fieldRelativeSpeeds.get().vxMetersPerSecond));
        return newPivotAngle;
    }

    // returns the pivot angle not accounting for movement
    public double getPivotAngleStationary(Pose3d pose) {
        double angleOnTheMove = fieldAngleToPose(pose);
        if (angleOnTheMove < 0) {
            angleOnTheMove += Math.PI;
        } else {
            angleOnTheMove -= Math.PI;
        }
        double angle = fieldAngleToPoseStationary(pose);
        if (angle < 0) {
            angle += Math.PI;
        } else {
            angle -= Math.PI;
        }
        var currentPose = compensatedPose();
        var pose3D =
                new Pose3d(
                        currentPose.getX(),
                        currentPose.getY(),
                        0,
                        new Rotation3d(0, 0, currentPose.getRotation().getRadians()));
        var shooterPose = pose3D.transformBy(robotToShooter);
        var shooter2D = shooterPose.toPose2d();
        double shooterDistance = GeomUtil.distance(pose.toPose2d().minus(shooter2D));
        double pivotAngle =
                Math.atan((pose.getZ() - shooterPose.getZ()) / shooterDistance)
                        + shooterDistance * Math.PI / 180 * 1;
        return pivotAngle;
    }

    // returns latency compensated pose;
    public Pose2d compensatedPose() {
        var speeds = fieldRelativeSpeeds.get();
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
        return 10;
    }

    // object storing doubles
    public class AimingParameters {
        public double rotation;
        public double pivot;

        public AimingParameters(double rotation, double pivot) {
            this.rotation = rotation;
            this.pivot = pivot;
        }
    }
}
