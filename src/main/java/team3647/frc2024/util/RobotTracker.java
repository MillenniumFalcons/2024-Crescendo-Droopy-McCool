package team3647.frc2024.util;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;
import team3647.lib.team6328.VirtualSubsystem;

public class RobotTracker extends VirtualSubsystem {

    private Pose2d speakerPose;
    private Pose2d ampPose;
    private Alliance color;

    private final Transform2d robotToShooter;

    private PeriodicIO periodicIO = new PeriodicIO();

    public static class PeriodicIO {
        private double distanceFromSpeaker = 0;
        private Pose2d pose = new Pose2d();
        private Pose2d compensatedPose = new Pose2d();
        private ChassisSpeeds speeds;
    }

    public RobotTracker(
            Pose2d speakerPose, Pose2d ampPose, Transform2d robotToShooter, boolean redSide) {
        this.speakerPose = speakerPose;
        this.ampPose = ampPose;
        this.robotToShooter = robotToShooter;
        this.color = redSide ? Alliance.Red : Alliance.Blue;
        if (redSide) {
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
        setCompensatedPose();
        setDistanceFromSpeaker();
        Logger.recordOutput("Robot/Compensated", periodicIO.compensatedPose);
        Logger.recordOutput("Robot/Pose", periodicIO.pose);
        Logger.recordOutput("Robot/Speeds", periodicIO.speeds.vxMetersPerSecond);
        // org.littletonrobotics.junction.Logger.recordOutput("speaker pose", speakerPose);
    }

    public void setCompensatedPose() {
        // final double shootSpeed = 15;
        // Pose2d pose = new Pose2d(periodicIO.pose.getX(), periodicIO.pose.getY(), new
        // Rotation2d());
        // double time = periodicIO.distanceFromSpeaker / shootSpeed;
        // var transform =
        //         new Twist2d(
        //                 periodicIO.speeds.vxMetersPerSecond * time,
        //                 periodicIO.speeds.vyMetersPerSecond * time,
        //                 0);
        // var newPose = pose.exp(transform);
        // periodicIO.compensatedPose = newPose;
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

    public double getCompensatedDistanceFromSpeaker() {
        return periodicIO.compensatedPose.minus(speakerPose).getTranslation().getNorm();
    }

    public double getDistanceFromSpeaker(Pose2d pose) {
        return pose.transformBy(robotToShooter).minus(speakerPose).getTranslation().getNorm();
    }

    public Pose2d getCompensatedPose() {
        return periodicIO.compensatedPose;
    }

    public Pose2d getPose() {
        return periodicIO.pose;
    }

    public void setStuff(SwerveDriveState state) {
        periodicIO.speeds = state.speeds;
        periodicIO.pose = state.Pose;
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
}
