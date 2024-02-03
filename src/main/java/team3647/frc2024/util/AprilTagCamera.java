package team3647.frc2024.util;

import java.util.Optional;

public interface AprilTagCamera {
    public Optional<VisionMeasurement> QueueToInputs();

    public int getTagNum();
}
