package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.lib.team6328.VirtualSubsystem;

public class RobotTracker extends VirtualSubsystem {

    private Pose2d speakerPose;
    private Pose2d ampPose;
    private Pose2d feedPose;
    private Alliance color;

    private InterpolatingDoubleTreeMap shootSpeedMap;

    Supplier<Pose2d> drivePose;
    Supplier<ChassisSpeeds> driveSpeeds;

    private final Transform2d robotToShooter;

    private PeriodicIO periodicIO = new PeriodicIO();

    public static class PeriodicIO {
        private double distanceFromSpeaker = 0;
        private Pose2d pose = new Pose2d();
        private ChassisSpeeds speeds;
    }

    public RobotTracker(
            Pose2d speakerPose,
            Pose2d ampPose,
            Pose2d feedPose,
            Transform2d robotToShooter,
            Supplier<Pose2d> drivePose,
            Supplier<ChassisSpeeds> driveSpeeds,
            InterpolatingDoubleTreeMap shootSpeedMap,
            boolean redSide) {
        this.speakerPose = speakerPose;
        this.ampPose = ampPose;
        this.robotToShooter = robotToShooter;
        this.feedPose = feedPose;
        this.drivePose = drivePose;
        this.driveSpeeds = driveSpeeds;
        this.shootSpeedMap = shootSpeedMap;
        this.color = redSide ? Alliance.Red : Alliance.Blue;
        if (redSide) {
            this.feedPose = AllianceFlip.flipForPP(feedPose);
            this.speakerPose = AllianceFlip.flipForPP(speakerPose);
            this.ampPose = AllianceFlip.flipForPP(ampPose);
        }

        // DriverStation.getAlliance()
        //         .ifPresent(
        //                 (alliance) ->
        //                         this.speakerPose =
        //                                 alliance == Alliance.Red
        //                                         ? AllianceFlip.flipForPP(speakerPose)
        //                                         : speakerPose);

        // DriverStation.getAlliance()
        //         .ifPresent(
        //                 (alliance) ->
        //                         this.ampPose =
        //                                 alliance == Alliance.Red
        //                                         ? AllianceFlip.flipForPP(ampPose)
        // : ampPose);
    }

    @Override
    public void periodic() {
        // setPose();
        setSpeeds();
        setPose();
        setDistanceFromSpeaker();
        Logger.recordOutput("Robot/Distance", getDistanceFromSpeaker());
        Logger.recordOutput("Robot/Pose", periodicIO.pose);
        // Logger.recordOutput("Robot/Speeds", periodicIO.speeds.vxMetersPerSecond);
        // org.littletonrobotics.junction.Logger.recordOutput("speaker pose", speakerPose);
    }

    public void setPose() {
        periodicIO.pose = drivePose.get();
    }

    public Pose2d compensate(Pose2d pose) {
        final double shootSpeed = 15 * (1 - periodicIO.distanceFromSpeaker / 40);

        double time = periodicIO.distanceFromSpeaker / shootSpeed;
        var transform =
                new Twist2d(
                        periodicIO.speeds.vxMetersPerSecond * time,
                        periodicIO.speeds.vyMetersPerSecond * time,
                        0);
        var middlePose = periodicIO.pose.exp(transform);
        double newDistance = getDistanceFromSpeaker(middlePose);
        double newSpeed = 15 * (1 - newDistance / 40);
        double newTime = newDistance / newSpeed;
        var newTransform =
                new Twist2d(
                        periodicIO.speeds.vxMetersPerSecond * newTime,
                        periodicIO.speeds.vyMetersPerSecond * newTime,
                        0);
        return pose.exp(newTransform);
    }

    public void setDistanceFromSpeaker() {
        var currentPose = periodicIO.pose;
        var shooterPose = currentPose.transformBy(robotToShooter);
        var shooterDistance = speakerPose.minus(shooterPose);
        periodicIO.distanceFromSpeaker = shooterDistance.getTranslation().getNorm();
    }

    public double getDistanceFromSpeaker() {
        return periodicIO.distanceFromSpeaker;
    }

    public double getDistanceFromSpeaker(Pose2d pose) {
        return pose.transformBy(robotToShooter).minus(speakerPose).getTranslation().getNorm();
    }

    public Pose2d getPose() {
        return periodicIO.pose;
    }

    public void setSpeeds() {
        periodicIO.speeds = driveSpeeds.get();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return periodicIO.speeds;
    }

    public Alliance getColor() {
        return color;
    }

    public Pose2d getAmp() {
        return this.ampPose;
    }

    public Pose2d getSpeaker() {
        return this.speakerPose;
    }

    public Pose2d getFeed() {
        return this.feedPose;
    }
}
