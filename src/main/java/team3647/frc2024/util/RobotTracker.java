package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2024.constants.FieldConstants;
import team3647.lib.team6328.VirtualSubsystem;

public class RobotTracker extends VirtualSubsystem implements AllianceUpdatedObserver {

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
            Transform2d robotToShooter) {
        this.speakerPose = speakerPose;
        this.ampPose = ampPose;
        this.drivePose = drivePose;
        this.robotRelativeSpeeds = robotRelativeSpeeds;
        this.robotToShooter = robotToShooter;

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
        Logger.recordOutput("target", speakerPose);
        // org.littletonrobotics.junction.Logger.recordOutput("speaker pose", speakerPose);
    }

    @Override
    public void onAllianceFound(Alliance alliance) {
        if (alliance == Alliance.Red) {
            this.speakerPose = AllianceFlip.flipForPP(FieldConstants.kBlueSpeaker);
            this.ampPose = AllianceFlip.flipForPP(FieldConstants.kBlueAmp);
        }
    }

    public void setCompensatedPose() {
        var speeds = robotRelativeSpeeds.get();
        var twist =
                new Twist2d(
                        speeds.vxMetersPerSecond * 0.02,
                        speeds.vyMetersPerSecond * 0.02,
                        speeds.omegaRadiansPerSecond * 0.02);
        periodicIO.compensatedPose = drivePose.get().exp(twist);
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
