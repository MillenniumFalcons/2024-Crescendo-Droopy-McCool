package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2024.constants.FieldConstants;
import team3647.frc2024.constants.PivotConstants;

public class TargetingUtil {

    private final RobotTracker robotTracker;
    private final double shootSpeed = 30;
    private double offset = 0;
    double kDt = 0.1;

    private final InterpolatingDoubleTreeMap speakerMap = PivotConstants.kMasterSpeakerMap;

    private final InterpolatingDoubleTreeMap ampMap = PivotConstants.kMasterAmpMap;

    public TargetingUtil(RobotTracker robotTracker) {
        this.robotTracker = robotTracker;
    }

    public Command offsetUp() {
        return Commands.runOnce(() -> offset += 1);
    }

    public Command offsetDown() {
        return Commands.runOnce(() -> offset -= 1);
    }

    // returns an object storing a pair of doubles, swerve angle change and pivot angle
    public AimingParameters shootAtPose(Pose3d pose) {
        double pivotAngle =
                Math.toRadians(speakerMap.get(robotTracker.getDistanceFromSpeaker()) + offset);
        if (pose.getZ() == FieldConstants.kAmpHeight) {
            pivotAngle =
                    Math.toRadians(
                            ampMap.get(robotTracker.getDistanceFromSpeaker())
                                    + offset); // pivot angle stationary
        }
        final var toPose =
                robotTracker.getCompensatedPose().minus(pose.toPose2d()).getTranslation();
        double angle =
                Math.acos(toPose.getX() / toPose.getNorm())
                        * Math.signum(toPose.getY())
                        * Math.signum(toPose.getX()); // field angle to pose stationary
        var newAngle =
                Math.atan(
                        (exitVelocity() * Math.cos(pivotAngle) * Math.sin(angle)
                                        + robotTracker.getChassisSpeeds().vyMetersPerSecond)
                                / (exitVelocity() * Math.cos(pivotAngle) * Math.cos(angle)
                                        + robotTracker.getChassisSpeeds().vxMetersPerSecond));
        boolean shouldAddPi = Math.cos(newAngle) < 0;
        double pi = shouldAddPi ? Math.PI : 0;
        boolean shouldSubtract = Math.sin(newAngle) < 0;
        pi = shouldSubtract ? -pi : pi;
        newAngle = newAngle + pi; // field angle to pose
        double robotAngleToPose =
                robotTracker.getCompensatedPose().getRotation().getRadians() - newAngle;
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
                                        - robotTracker.getChassisSpeeds().vxMetersPerSecond));
        return new AimingParameters(robotAngleToPose, newPivotAngle, shootSpeed);
    }

    public AimingParameters shootAtPose(Pose3d pose, double shootSpeed) {
        return shootAtPose(pose).withShootSpeed(shootSpeed);
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtSpeaker() {
        return shootAtPose(robotTracker.getSpeaker());
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtAmp() {
        return shootAtPose(robotTracker.getAmp(), 5.5);
    }

    public double distance() {
        return robotTracker.getDistanceFromSpeaker();
    }

    public Pose2d compensatedPose() {
        return robotTracker.getCompensatedPose();
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
