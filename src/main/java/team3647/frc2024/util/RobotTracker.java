package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import team3647.lib.team6328.VirtualSubsystem;

public class RobotTracker extends VirtualSubsystem {

    private Pose2d speakerPose;
    private Pose2d ampPose;

    private final Supplier<Pose2d> drivePose;
    private final Supplier<ChassisSpeeds> robotRelativeSpeeds;

    private final Transform2d robotToShooter;

    private PeriodicIO periodicIO = new PeriodicIO();

    public static class PeriodicIO {
        private double distanceFromSpeaker = 0;
        private Pose2d compensatedPose = new Pose2d();
    }

    public RobotTracker(
            Pose2d speakerPose,
            Pose2d ampPose,
            Supplier<Pose2d> drivePose,
            Supplier<ChassisSpeeds> robotRelativeSpeeds,
            Transform2d robotToShooter,
            boolean redSide) {
        this.speakerPose = speakerPose;
        this.ampPose = ampPose;
        this.drivePose = drivePose;
        this.robotRelativeSpeeds = robotRelativeSpeeds;
        this.robotToShooter = robotToShooter;
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
        // org.littletonrobotics.junction.Logger.recordOutput("speaker pose", speakerPose);
    }

    public void setCompensatedPose() {
        periodicIO.compensatedPose = drivePose.get();
    }

    public void setDistanceFromSpeaker() {
        var currentPose = periodicIO.compensatedPose;
        var shooterPose = currentPose.transformBy(robotToShooter);
        var shooterDistance = speakerPose.minus(shooterPose);
        periodicIO.distanceFromSpeaker = shooterDistance.getTranslation().getNorm();
    }

    public double getDistanceFromSpeaker() {
        return periodicIO.distanceFromSpeaker;
    }

    public Pose2d getCompensatedPose() {
        return periodicIO.compensatedPose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.robotRelativeSpeeds.get();
    }

    public Pose2d getAmp() {
        return this.ampPose;
    }

    public Pose2d getSpeaker() {
        return this.speakerPose;
    }
}
