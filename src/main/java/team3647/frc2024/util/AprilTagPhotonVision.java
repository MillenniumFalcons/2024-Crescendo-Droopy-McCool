package team3647.frc2024.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import team3647.frc2024.constants.FieldConstants;
import team3647.lib.vision.AprilTagCamera.AprilTagId;

public class AprilTagPhotonVision extends PhotonCamera implements AprilTagCamera {

    AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPoseEstimator photonPoseEstimator;
    Transform3d robotToCam;
    private final edu.wpi.first.math.Vector<N3> baseStdDevs;
    private final edu.wpi.first.math.Vector<N3> multiStdDevs =
            VecBuilder.fill(0.00096, 0.00096, 0.02979);
    private boolean hasPriority = false;
    private final String name;

    public AprilTagPhotonVision(
            String camera, Transform3d robotToCam, edu.wpi.first.math.Vector<N3> baseStdDevs) {
        super(NetworkTableInstance.getDefault(), camera);
        this.name = camera;
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        this,
                        robotToCam);
        this.baseStdDevs = baseStdDevs;
        this.robotToCam = robotToCam;
    }

    public AprilTagPhotonVision(String camera, Transform3d robotToCam) {
        this(camera, robotToCam, VecBuilder.fill(0.05, 0.05, 5));
    }

    public AprilTagId getId(int id) {
        var adjustedId = id - 1;
        var possibleValues = AprilTagId.values();
        if (adjustedId < 0 || adjustedId > possibleValues.length) {
            return AprilTagId.ID_DNE;
        }

        return possibleValues[adjustedId];
    }

    public AprilTagPhotonVision withPriority(boolean priority) {
        this.hasPriority = true;
        return this;
    }

    public String getName() {
        return this.name;
    }

    public Optional<Pose3d> camPose() {
        var update = photonPoseEstimator.update();
        if (update.isEmpty()) {
            return Optional.empty();
        }
        var bruh = update.get().estimatedPose;
        return Optional.of(bruh.transformBy(robotToCam));
    }

    public Optional<VisionMeasurement> QueueToInputs() {
        var result = this.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        var update = photonPoseEstimator.update(result);
        if (update.isEmpty()) {
            return Optional.empty();
        }
        double targetDistance =
                result.getBestTarget()
                        .getBestCameraToTarget()
                        .getTranslation()
                        .toTranslation2d()
                        .getNorm();
        if (hasPriority) {
            targetDistance = MathUtil.clamp(targetDistance - 3, 1, 1000);
        }
        // if (result.getBestTarget().getFiducialId() != 3
        //         && result.getBestTarget().getFiducialId() != 4) {
        //     return Optional.empty();
        // }
        if (targetDistance > 4 && !hasPriority) {
            return Optional.empty();
        }
        if (targetDistance > 8) {
            return Optional.empty();
        }
        if (Math.abs(update.get().estimatedPose.getZ()) > 0.5) {
            return Optional.empty();
        }
        if (update.get().estimatedPose.getX() > FieldConstants.kFieldLength
                || update.get().estimatedPose.getX() < 0
                || update.get().estimatedPose.getY() > FieldConstants.kFieldWidth
                || update.get().estimatedPose.getY() < 0) {
            return Optional.empty();
        }

        // if (result.getBestTarget().getFiducialId() == 5
        //         || result.getBestTarget().getFiducialId() == 6) {
        //     return Optional.empty();
        // }
        // Logger.recordOutput(
        //         "Cams/" + this.getName(), update.get().estimatedPose.transformBy(robotToCam));
        double numTargets = result.getTargets().size();
        if (hasPriority && numTargets < 2) {
            return Optional.empty();
        }
        final var stdDevs = baseStdDevs.times(targetDistance).times(8 / Math.pow(numTargets, 3));
        double ambiguityScore =
                1 / (numTargets * 100 + (1 - result.getBestTarget().getPoseAmbiguity()));

        if (hasPriority) {
            Logger.recordOutput("Zoom ambiguity schore", ambiguityScore);
        }

        // final double priorityScore = this.hasPriority ? 50 : 0;
        // ambiguityScore += priorityScore;

        if (result.targets.stream().anyMatch(target -> target.getPoseAmbiguity() > 0.2)) {

            return Optional.empty();
        }
        if (hasPriority) {
            Logger.recordOutput(
                    "toomuchambiguity?",
                    result.targets.stream().anyMatch(t -> t.getPoseAmbiguity() > 0.2));
        }
        VisionMeasurement measurement =
                VisionMeasurement.fromEstimatedRobotPose(
                        update.get(),
                        update.get().timestampSeconds,
                        ambiguityScore,
                        stdDevs,
                        getName());
        return Optional.of(measurement);
    }

    public int getTagNum() {
        var result = this.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getFiducialId();
        } else {
            return -1;
        }
    }
}
