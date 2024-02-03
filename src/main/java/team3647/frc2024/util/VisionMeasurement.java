package team3647.frc2024.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Objects;
import org.photonvision.EstimatedRobotPose;

public class VisionMeasurement {
    public final Pose2d pose;
    public final double timestamp;
    public final Matrix<N3, N1> stdDevs;

    public VisionMeasurement(Pose2d pose, double timestamp) {
        this(pose, timestamp, VecBuilder.fill(0.05, 0.05, 0.05));
    }

    public VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        Objects.requireNonNull(stdDevs);
        this.pose = pose;
        this.timestamp = timestamp;
        this.stdDevs = stdDevs;
    }

    public static VisionMeasurement fromEstimatedRobotPose(
            EstimatedRobotPose estimatedPose, Vector<N3> stdDevs) {
        return new VisionMeasurement(
                estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, stdDevs);
    }
}
