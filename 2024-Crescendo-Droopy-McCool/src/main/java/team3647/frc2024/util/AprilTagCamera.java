package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

public interface AprilTagCamera {
    public Optional<VisionMeasurement> QueueToInputs();

    public int getTagNum();

    public String getName();

    public Optional<Pose3d> camPose();
}
