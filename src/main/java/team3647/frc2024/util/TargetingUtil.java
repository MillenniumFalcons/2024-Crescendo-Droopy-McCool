package team3647.frc2024.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import team3647.lib.GeomUtil;

public class TargetingUtil {

    private final Pose3d speakerPose;
    private final Pose3d ampPose;
    private final Supplier<Pose2d> drivePose;
    private final Supplier<ChassisSpeeds> robotRelativeSpeeds;
    private final Transform3d robotToShooter;
    private final double shootSpeed = 10;
    private double offset = 2;
    double kDt = 0.02;

    public TargetingUtil(
            Pose3d speakerPose,
            Pose3d ampPose,
            Supplier<Pose2d> drivePose,
            Supplier<ChassisSpeeds> robotRelativeSpeeds,
            Transform3d robotToShooter) {
        this.speakerPose = speakerPose;
        this.ampPose = ampPose;
        this.drivePose = drivePose;
        this.robotRelativeSpeeds = robotRelativeSpeeds;
        this.robotToShooter = robotToShooter;
    }

    public Command offsetUp() {
        return Commands.runOnce(() -> offset += 0.1);
    }

    public Command offsetDown() {
        return Commands.runOnce(() -> offset -= 0.1);
    }

    // returns an object storing a pair of doubles, swerve angle change and pivot angle
    public AimingParameters shootAtPose(Pose3d pose) {
        return new AimingParameters(robotAngleToPose(pose), getPivotAngle(pose));
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtSpeaker() {
        return shootAtPose(speakerPose);
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtAmp() {
        return shootAtPose(ampPose);
    }

    // returns the angle between the vector pointing straight back and the pose
    public double fieldAngleToPose(Pose3d pose) {
        final var currentPose = compensatedPose();
        final var toPose =
                VecBuilder.fill(pose.getX() - currentPose.getX(), pose.getY() - currentPose.getY());
        double angle =
                Math.acos(toPose.dot(VecBuilder.fill(-1, 0)) / toPose.norm())
                        * Math.signum(currentPose.getY() - pose.getY())
                        * Math.signum(currentPose.getX() - pose.getX());
        var newAngle =
                Math.atan(
                        (exitVelocity() * Math.cos(getPivotAngleStationary(pose)) * Math.sin(angle)
                                        + robotRelativeSpeeds.get().vyMetersPerSecond)
                                / (exitVelocity()
                                                * Math.cos(getPivotAngleStationary(pose))
                                                * Math.cos(angle)
                                        + robotRelativeSpeeds.get().vxMetersPerSecond));
        boolean shouldAddPi = Math.cos(newAngle) < 0;
        double pi = shouldAddPi ? Math.PI : 0;
        boolean shouldSubtract = Math.sin(newAngle) < 0;
        pi = shouldSubtract ? -pi : pi;
        newAngle = newAngle + pi;
        SmartDashboard.putNumber("new anlge", newAngle);
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
                        * Math.signum(currentPose.getY() - pose.getY())
                        * Math.signum(currentPose.getX() - pose.getX());
        return angle;
    }

    // return the angle between the robots back heading (since shooter is at back) and the pose
    public double robotAngleToPose(Pose3d pose) {
        final var currentPose = compensatedPose();
        var rot = currentPose.getRotation().getRadians();
        SmartDashboard.putNumber("rot", rot);
        final var fieldAngle = fieldAngleToPose(pose);
        var newAngle = rot - fieldAngle;

        double distance = GeomUtil.distance(pose.toPose2d().minus(currentPose));

        return newAngle - 0.01 * distance;
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
                        + (shooterDistance + 0.3) * Math.PI / 180 * offset;
        double newPivotAngle =
                Math.atan(
                        (exitVelocity() * Math.sin(pivotAngle) * Math.cos(angle))
                                / (exitVelocity() * Math.cos(pivotAngle) * Math.cos(angleOnTheMove)
                                        - robotRelativeSpeeds.get().vxMetersPerSecond));
        SmartDashboard.putNumber("new pibotr angle", newPivotAngle * 180 / Math.PI);
        return newPivotAngle;
    }

    // returns the pivot angle not accounting for movement
    public double getPivotAngleStationary(Pose3d pose) {
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
                        + (shooterDistance + 0.3) * Math.PI / 180 * offset;
        return pivotAngle;
    }

    public double getPivotAngleByPose(Pose2d botPose) {
        var currentPose = botPose;
        var pose3D =
                new Pose3d(
                        currentPose.getX(),
                        currentPose.getY(),
                        0,
                        new Rotation3d(0, 0, currentPose.getRotation().getRadians()));
        var shooterPose = pose3D.transformBy(robotToShooter);
        var shooter2D = shooterPose.toPose2d();
        var pose = speakerPose;
        double shooterDistance = GeomUtil.distance(pose.toPose2d().minus(shooter2D));
        double pivotAngle =
                Math.atan((pose.getZ() - shooterPose.getZ()) / shooterDistance)
                        + (shooterDistance) * Math.PI / 180 * offset;
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

        public AimingParameters(double rotation, double pivot) {
            this.rotation = rotation;
            this.pivot = pivot;
        }
    }
}