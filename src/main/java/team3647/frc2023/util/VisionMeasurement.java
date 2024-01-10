package team3647.frc2023.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionMeasurement {
    public Pose2d pose;
    public double timestamp;
    public Matrix<N3, N1> stdDevs = VecBuilder.fill(0.01, 0.01, 0.01);

    public VisionMeasurement(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }

    public VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.stdDevs = stdDevs;
    }
}
