package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final BooleanSupplier turnOffVision;

    public VisionController(
            Consumer<VisionMeasurement> visionAcceptor,
            Function<Pose2d, Boolean> shouldAddData,
            BooleanSupplier dataAddOverride,
            BooleanSupplier turnOffVision,
            AprilTagCamera... cameras) {
        this.cameras = cameras;
        this.shouldAddData = shouldAddData;
        this.botPoseAcceptor = visionAcceptor;
        this.dataAddOverride = dataAddOverride;
        this.turnOffVision = turnOffVision;
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

            // Logger.recordOutput("Robot/" + camera.getName(), getInputs.pose);
        }

        if (!list.isEmpty() && !turnOffVision.getAsBoolean()) {

            Collections.sort(list);

            botPoseAcceptor.accept(list.get(0));

            Logger.recordOutput("Robot/Vision", list.get(0).pose);
            Logger.recordOutput("Robot/Camera", list.get(0).name);
            Logger.recordOutput("stddev", list.get(0).stdDevs.get(0, 0));
        } else {
            Logger.recordOutput("Robot/Vision", new Pose2d(1, 1, new Rotation2d()));
            Logger.recordOutput("stddev", 0.0);
        }
    }
}
