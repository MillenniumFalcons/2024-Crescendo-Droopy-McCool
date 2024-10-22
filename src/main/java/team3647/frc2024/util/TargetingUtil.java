package team3647.frc2024.util;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2024.constants.PivotConstants;

public class TargetingUtil {

    private final RobotTracker robotTracker;
    private final double shootSpeed = 30;
    private double offset = 0.5;
    double kDt = 0.1;

    private final MedianFilter filter = new MedianFilter(3);

    private final InterpolatingDoubleTreeMap speakerMap = PivotConstants.kMasterSpeakerMap;

    private final InterpolatingDoubleTreeMap ampMap = PivotConstants.kMasterAmpMap;

    public TargetingUtil(RobotTracker robotTracker) {
        this.robotTracker = robotTracker;
    }

    public Command offsetUp() {
        return Commands.runOnce(() -> offset += 0.2);
    }

    public Command offsetDown() {
        return Commands.runOnce(() -> offset -= 0.2);
    }

    public double getOffset() {
        return offset;
    }

    // returns an object storing a pair of doubles, swerve angle change and pivot angle
    public AimingParameters shootAtPoseOnTheMove(Pose2d pose) {
        Pose2d shotPose = robotTracker.compensate(robotTracker.getPose());
        double pivotAngle =
                Math.toRadians(
                        speakerMap.get(robotTracker.getDistanceFromSpeaker(shotPose)) + offset);
        final var toPose = shotPose.minus(pose).getTranslation();
        double angle =
                Math.acos(toPose.getX() / toPose.getNorm())
                        * Math.signum(toPose.getY())
                        * Math.signum(toPose.getX()); // field angle to pose stationary

        double invert = 1;

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                angle -= Math.PI;
                invert = -1;
            }
        }
        var newAngle = angle;
        // Math.atan2(
        //         (adjustedExit() * Math.cos(pivotAngle) * Math.sin(angle)
        //                 + robotTracker.getChassisSpeeds().vyMetersPerSecond * invert),
        //         (adjustedExit() * Math.cos(pivotAngle) * Math.cos(angle)
        //                 + robotTracker.getChassisSpeeds().vxMetersPerSecond * invert));
        // SmartDashboard.putNumber("new anle", newAngle);
        // boolean shouldAddPi = Math.cos(newAngle) < 0;
        // double pi = shouldAddPi ? Math.PI : 0;
        // boolean shouldSubtract = Math.sin(newAngle) < 0;
        // pi = shouldSubtract ? -pi : pi;
        // newAngle = newAngle + pi; // field angle to pose
        double robotAngleToPose = robotTracker.getPose().getRotation().getRadians() - newAngle;
        if (robotAngleToPose > Math.PI) {
            robotAngleToPose -= 2 * Math.PI;
        }
        if (robotAngleToPose < -Math.PI) {
            robotAngleToPose += 2 * Math.PI;
        }
        // SmartDashboard.putNumber("roobot angle", robotAngleToPose);
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
        double newPivotAngle = pivotAngle;
        // Math.atan(
        //         (adjustedExit() * Math.sin(pivotAngle) * Math.cos(angleStationary))
        //                 / (adjustedExit() * Math.cos(pivotAngle) * Math.cos(angleOnTheMove)
        //                         - robotTracker.getChassisSpeeds().vxMetersPerSecond
        //                                 * invert));
        return new AimingParameters(robotAngleToPose, newPivotAngle, shootSpeed);
    }

    public AimingParameters feedOnTheMove(Pose2d pose) {
        Pose2d shotPose = robotTracker.compensateFeed(robotTracker.getPose());
        double pivotAngle =
                Math.toRadians(
                        PivotConstants.feedmap.get(robotTracker.getDistanceFromSpeaker(shotPose))
                                + offset);
        final var toPose = shotPose.minus(pose).getTranslation();
        double angle =
                Math.acos(toPose.getX() / toPose.getNorm())
                        * Math.signum(toPose.getY())
                        * Math.signum(toPose.getX()); // field angle to pose stationary

        double invert = 1;

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                angle -= Math.PI;
                invert = -1;
            }
        }
        var newAngle = angle;
        // Math.atan2(
        //         (adjustedExit() * Math.cos(pivotAngle) * Math.sin(angle)
        //                 + robotTracker.getChassisSpeeds().vyMetersPerSecond * invert),
        //         (adjustedExit() * Math.cos(pivotAngle) * Math.cos(angle)
        //                 + robotTracker.getChassisSpeeds().vxMetersPerSecond * invert));
        // SmartDashboard.putNumber("new anle", newAngle);
        // boolean shouldAddPi = Math.cos(newAngle) < 0;
        // double pi = shouldAddPi ? Math.PI : 0;
        // boolean shouldSubtract = Math.sin(newAngle) < 0;
        // pi = shouldSubtract ? -pi : pi;
        // newAngle = newAngle + pi; // field angle to pose
        double robotAngleToPose = robotTracker.getPose().getRotation().getRadians() - newAngle;
        if (robotAngleToPose > Math.PI) {
            robotAngleToPose -= 2 * Math.PI;
        }
        if (robotAngleToPose < -Math.PI) {
            robotAngleToPose += 2 * Math.PI;
        }
        // SmartDashboard.putNumber("roobot angle", robotAngleToPose);
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
        double newPivotAngle = pivotAngle;
        // Math.atan(
        //         (adjustedExit() * Math.sin(pivotAngle) * Math.cos(angleStationary))
        //                 / (adjustedExit() * Math.cos(pivotAngle) * Math.cos(angleOnTheMove)
        //                         - robotTracker.getChassisSpeeds().vxMetersPerSecond
        //                                 * invert));
        return new AimingParameters(robotAngleToPose, newPivotAngle, shootSpeed);
    }

    // returns an object storing a pair of doubles, swerve angle change and pivot angle
    public AimingParameters shootAtPose(Pose2d pose) {
        double pivotAngle =
                Math.toRadians(speakerMap.get(robotTracker.getDistanceFromSpeaker()) + offset);
        final var toPose = robotTracker.getPose().minus(pose).getTranslation();
        double angle =
                Math.acos(toPose.getX() / toPose.getNorm())
                        * Math.signum(toPose.getY())
                        * Math.signum(toPose.getX()); // field angle to pose stationary

        double invert = 1;

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                angle -= Math.PI;
                invert = -1;
            }
        }
        var newAngle = angle;
        // Math.atan2(
        //         (adjustedExit() * Math.cos(pivotAngle) * Math.sin(angle)
        //                 + robotTracker.getChassisSpeeds().vyMetersPerSecond * invert),
        //         (adjustedExit() * Math.cos(pivotAngle) * Math.cos(angle)
        //                 + robotTracker.getChassisSpeeds().vxMetersPerSecond * invert));
        // SmartDashboard.putNumber("new anle", newAngle);
        // boolean shouldAddPi = Math.cos(newAngle) < 0;
        // double pi = shouldAddPi ? Math.PI : 0;
        // boolean shouldSubtract = Math.sin(newAngle) < 0;
        // pi = shouldSubtract ? -pi : pi;
        // newAngle = newAngle + pi; // field angle to pose
        double robotAngleToPose = robotTracker.getPose().getRotation().getRadians() - newAngle;
        if (robotAngleToPose > Math.PI) {
            robotAngleToPose -= 2 * Math.PI;
        }
        if (robotAngleToPose < -Math.PI) {
            robotAngleToPose += 2 * Math.PI;
        }
        // SmartDashboard.putNumber("roobot angle", robotAngleToPose);
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
        double newPivotAngle = pivotAngle;
        // Math.atan(
        //         (adjustedExit() * Math.sin(pivotAngle) * Math.cos(angleStationary))
        //                 / (adjustedExit() * Math.cos(pivotAngle) * Math.cos(angleOnTheMove)
        //                         - robotTracker.getChassisSpeeds().vxMetersPerSecond
        //                                 * invert));
        return new AimingParameters(robotAngleToPose, newPivotAngle, shootSpeed);
    }

    public double getCompensatedDistance() {
        return robotTracker.getDistanceFromSpeaker(robotTracker.compensate(robotTracker.getPose()));
    }

    public Pose2d getCompensatedPose() {
        return robotTracker.compensate(robotTracker.getPose());
    }

    public AimingParameters shootAtPose(Pose2d pose, double shootSpeed) {
        return shootAtPose(pose).withShootSpeed(shootSpeed);
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtSpeaker() {
        return shootAtPose(robotTracker.getSpeaker());
    }

    public AimingParameters shootAtFeed() {
        return feedOnTheMove(robotTracker.getFeed());
    }

    public AimingParameters shootAtSpeakerOnTheMove() {
        return shootAtPoseOnTheMove(robotTracker.getSpeaker());
    }

    // returns the parameters for aiming at the speaker
    public AimingParameters shootAtAmp() {
        return shootAtPose(robotTracker.getAmp(), 5.5);
    }

    public double distance() {
        return robotTracker.getDistanceFromSpeaker();
    }

    public Pose2d pose() {
        return robotTracker.getPose();
    }

    public double rotToAmp() {
        if (robotTracker.getPose().getRotation().getRadians() >= Math.PI / 2) {
            return -Math.PI / 2 - (robotTracker.getPose().getRotation().getRadians() - 2 * Math.PI);
        } else {
            return -Math.PI / 2 - robotTracker.getPose().getRotation().getRadians();
        }
    }

    public double rotToOther90() {
        if (robotTracker.getPose().getRotation().getRadians() <= -Math.PI / 2) {
            return Math.PI / 2 - (robotTracker.getPose().getRotation().getRadians() + 2 * Math.PI);
        } else {
            return Math.PI / 2 - robotTracker.getPose().getRotation().getRadians();
        }
    }

    public double getAmpX() {
        return robotTracker.getAmp().getX();
    }

    public double getAmpY() {
        return robotTracker.getAmp().getY();
    }

    // returns shot exit velocity
    public double exitVelocity() {
        return shootSpeed;
    }

    public double adjustedExit() {
        return shootSpeed * 2 / 5;
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
