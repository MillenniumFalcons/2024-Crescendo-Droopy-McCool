package team3647.frc2024.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Objects;
import org.photonvision.EstimatedRobotPose;

public class VisionMeasurement implements Comparable<VisionMeasurement> {
    public final Pose2d pose;
    public final double timestamp;
    public final double ambiguityScore;
    public final Matrix<N3, N1> stdDevs;
    public final String name;

    public VisionMeasurement(Pose2d pose, double timestamp) {
        this(pose, timestamp, 1, VecBuilder.fill(0.05, 0.05, 0.05), "none");
    }

    public VisionMeasurement(
            Pose2d pose,
            double timestamp,
            double ambiguityScore,
            Matrix<N3, N1> stdDevs,
            String name) {
        Objects.requireNonNull(stdDevs);
        this.ambiguityScore = ambiguityScore;
        this.pose = pose;
        this.timestamp = timestamp;
        this.stdDevs = stdDevs;
        this.name = name;
    }

    public static VisionMeasurement fromEstimatedRobotPose(
            EstimatedRobotPose estimatedPose,
            double ambiguityScore,
            Vector<N3> stdDevs,
            String name) {
        return new VisionMeasurement(
                estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds,
                ambiguityScore,
                stdDevs,
                name);
    }

    public static VisionMeasurement fromEstimatedRobotPose(
            EstimatedRobotPose estimatedPose,
            double timestampSeconds,
            double ambiguityScore,
            Vector<N3> stdDevs,
            String name) {
        return new VisionMeasurement(
                estimatedPose.estimatedPose.toPose2d(),
                timestampSeconds,
                ambiguityScore,
                stdDevs,
                name);
    }

    @Override
    public int compareTo(VisionMeasurement o) {
        return Double.compare(this.ambiguityScore, o.ambiguityScore);
    }
}
