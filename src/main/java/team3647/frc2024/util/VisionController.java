package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import team3647.lib.team6328.VirtualSubsystem;

public class VisionController extends VirtualSubsystem {
    private final AprilTagCamera[] cameras;
    private final Consumer<VisionMeasurement> botPoseAcceptor;
    private final Function<Pose2d, Boolean> shouldAddData;
    private final ArrayList<VisionMeasurement> list = new ArrayList<>();
    private final BooleanSupplier dataAddOverride;

    public VisionController(
            Consumer<VisionMeasurement> visionAcceptor,
            Function<Pose2d, Boolean> shouldAddData,
            BooleanSupplier dataAddOverride,
            AprilTagCamera... cameras) {
        this.cameras = cameras;
        this.shouldAddData = shouldAddData;
        this.botPoseAcceptor = visionAcceptor;
        this.dataAddOverride = dataAddOverride;
    }

    @Override
    public void periodic() {

        list.clear();

        for (AprilTagCamera camera : cameras) {

            var inputs = camera.QueueToInputs();

            if (inputs.isEmpty()) {
                continue;
            }

            var getInputs = inputs.get();

            if (shouldAddData.apply(getInputs.pose) || dataAddOverride.getAsBoolean()) {
                list.add(getInputs);
            }
        }

        if (!list.isEmpty()) {

            Collections.sort(list);

            botPoseAcceptor.accept(list.get(0));

            Logger.recordOutput("Robot/Vision", list.get(0).pose);
        }
    }
}
