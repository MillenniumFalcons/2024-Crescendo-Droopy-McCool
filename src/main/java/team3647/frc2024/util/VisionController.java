package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.lib.GeomUtil;
import team3647.lib.team6328.VirtualSubsystem;

public class VisionController extends VirtualSubsystem {
    private final AprilTagCamera[] cameras;
    private final Consumer<VisionMeasurement> botPoseAcceptor;
    private final SwerveDrive swerve;

    public VisionController(
            Consumer<VisionMeasurement> visionAcceptor,
            SwerveDrive swerve,
            AprilTagCamera... cameras) {
        this.cameras = cameras;
        this.swerve = swerve;
        this.botPoseAcceptor = visionAcceptor;
    }

    @Override
    public void periodic() {

        for (int i = 0; i < cameras.length; i++) {

            if (cameras[i].getTagNum() == -1) {
                continue;
            }

            var inputs = cameras[i].QueueToInputs();

            if (inputs.isEmpty()) {
                break;
            }

            var getInputs = inputs.get();

            Logger.recordOutput("Robot/Vision", getInputs.pose);

            if (shouldAddData(getInputs.pose, swerve::getOdoPose)) {
                botPoseAcceptor.accept(
                        new VisionMeasurement(
                                getInputs.pose, Timer.getFPGATimestamp(), getInputs.stdDevs));
            }
        }
    }

    public boolean shouldAddData(Pose2d visionPose, Supplier<Pose2d> drivePose) {
        var pose = drivePose.get();
        double distance = GeomUtil.distance(visionPose, pose);
        return (distance < 1) ? true : false;
    }
}
