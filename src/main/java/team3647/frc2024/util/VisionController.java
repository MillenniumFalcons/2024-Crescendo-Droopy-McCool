package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Consumer;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import team3647.frc2024.constants.VisionConstants;
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
                continue;
            }

            var getInputs = inputs.get();

            // Logger.recordOutput("Robot/Vision/" + camera.getName(), getInputs.pose);

            // Logger.recordOutput("Test/Bot", VisionConstants.randompose);
            // Logger.recordOutput(
            //         "Test/BackRight",
            //         VisionConstants.randompose.transformBy(VisionConstants.robotToBackRight));
            // Logger.recordOutput(
            //         "Test/BackLeft",
            //         VisionConstants.randompose.transformBy(VisionConstants.robotToBackLeft));
            // Logger.recordOutput(
            //         "Test/Right",
            //         VisionConstants.randompose.transformBy(VisionConstants.robotToRight));
            // Logger.recordOutput(
            //         "Test/Left",
            //         VisionConstants.randompose.transformBy(VisionConstants.robotToLeft));

            if (shouldAddData.apply(getInputs.pose)) {
                botPoseAcceptor.accept(
                        new VisionMeasurement(
                                getInputs.pose, getInputs.timestamp, getInputs.stdDevs));
            }
        }
    }
}
