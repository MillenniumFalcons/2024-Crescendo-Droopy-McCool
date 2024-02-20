package team3647.frc2024.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import team3647.lib.vision.AprilTagCamera.AprilTagId;

public class AprilTagPhotonVision extends PhotonCamera implements AprilTagCamera {

    AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPoseEstimator photonPoseEstimator;
    Transform3d robotToCam;

    public AprilTagPhotonVision(String camera, Transform3d robotToCam) {
        super(NetworkTableInstance.getDefault(), camera);
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        this,
                        robotToCam);
        this.robotToCam = robotToCam;
    }

    public static AprilTagId getId(int id) {
        var adjustedId = id - 1;
        var possibleValues = AprilTagId.values();
        if (adjustedId < 0 || adjustedId > possibleValues.length) {
            return AprilTagId.ID_DNE;
        }

        return possibleValues[adjustedId];
    }

    public Optional<VisionMeasurement> QueueToInputs() {
        var update = photonPoseEstimator.update();
        var result = this.getLatestResult();
        if (update.isEmpty() || !result.hasTargets()) {
            return Optional.empty();
        }
        var targetDistance =
                result.getBestTarget()
                        .getBestCameraToTarget()
                        .getTranslation()
                        .toTranslation2d()
                        .getNorm();
        if (targetDistance > 2.5) {
            return Optional.empty();
        }
        final var stdDevs = VecBuilder.fill(0.1, 0.1, 0.1).times(targetDistance);
        VisionMeasurement measurement =
                VisionMeasurement.fromEstimatedRobotPose(update.get(), stdDevs);
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
