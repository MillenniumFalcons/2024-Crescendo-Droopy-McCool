package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Consumer;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import team3647.lib.team6328.VirtualSubsystem;

public class VisionController extends VirtualSubsystem {
    private final AprilTagCamera[] cameras;
    private final Consumer<VisionMeasurement> botPoseAcceptor;
    private final Function<Pose2d, Boolean> shouldAddData;

    public VisionController(
            Consumer<VisionMeasurement> visionAcceptor,
            Function<Pose2d, Boolean> shouldAddData,
            AprilTagCamera... cameras) {
        this.cameras = cameras;
        this.shouldAddData = shouldAddData;
        this.botPoseAcceptor = visionAcceptor;
    }

    @Override
    public void periodic() {

        for (AprilTagCamera camera : cameras) {

            var inputs = camera.QueueToInputs();

            if (inputs.isEmpty()) {
                break;
            }

            var getInputs = inputs.get();

            Logger.recordOutput("Robot/Vision", getInputs.pose);

            if (shouldAddData.apply(getInputs.pose)) {
                botPoseAcceptor.accept(
                        new VisionMeasurement(
                                getInputs.pose, getInputs.timestamp, getInputs.stdDevs));
            }
        }
    }
}
