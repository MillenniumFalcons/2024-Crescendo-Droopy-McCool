package team3647.frc2023.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.lib.team6328.VirtualSubsystem;

public class VisionController extends VirtualSubsystem {
    private final AprilTagCamera[] cameras;
    private final Consumer<VisionMeasurement> botPoseAcceptor;
    private final SwerveDrive swerve;
    // private final Supplier<Twist2d> driveData;
    private final double kDt = 0.02;
    private final double allowedError = 0.15;

    public VisionController(
            Consumer<VisionMeasurement> visionAcceptor,
            SwerveDrive swerve,
            // Supplier<Twist2d> driveData,
            AprilTagCamera... cameras) {
        this.cameras = cameras;
        this.swerve = swerve;
        // this.driveData = driveData;
        this.botPoseAcceptor = visionAcceptor;
    }

    @Override
    public void periodic() {

        double avgX = 0;
        double avgY = 0;
        double avgRot = 0;
        double latestTimestamp = 0;
        Matrix<N3, N1> latestMatrix = VecBuilder.fill(0, 0, 0);
        int count = 0;

        for (int i = 0; i < cameras.length; i++) {

            // if (!cameras[i].QueueToInputs().isPresent()) {
            //     continue;
            // }

            if (cameras[i].getTagNum() == -1) {
                continue;
            }

            var inputs = cameras[i].QueueToInputs();

            if (inputs.isEmpty()) {
                break;
            }

            for (int j = 0; j < inputs.get().length; j++) {

                // var expectedPose =
                //         swerve.getPoseByTimestamp(inputs[j].timestamp - kDt)
                //                 .transformBy(
                //                         swerve.getPoseByTimestamp(inputs[j].timestamp - kDt)
                //                                 .minus(
                //                                         swerve.getPoseByTimestamp(
                //                                                 inputs[j].timestamp - kDt * 2)));
                // if (shouldAddData(inputs[j].pose, expectedPose)) {
                if (inputs.get()[j] != null
                        && inputs.get()[j].pose.getX() > 0.05
                        && inputs.get()[j].pose.getY() > 0.05) {
                    // SmartDashboard.putData("null", null);
                    avgX += inputs.get()[j].pose.getX();
                    avgY += inputs.get()[j].pose.getY();
                    avgRot += inputs.get()[j].pose.getRotation().getRadians();
                    count++;
                    latestTimestamp = inputs.get()[j].timestamp;
                    latestMatrix = inputs.get()[j].stdDevs;
                }
                // }
            }
        }

        if (count > 0) {
            avgX /= count;
            avgY /= count;
            avgRot /= count;
            Logger.recordOutput("Robot/Vision", new Pose2d(avgX, avgY, new Rotation2d(avgRot)));
            // SmartDashboard.putNumber("rot vision", avgRot);
            // if (shouldAddData(new Pose2d(avgX, avgY, new Rotation2d(-avgRot)),
            // swerve.getOdoPose()))
            botPoseAcceptor.accept(
                    new VisionMeasurement(
                            new Pose2d(avgX, avgY, new Rotation2d(avgRot)),
                            latestTimestamp,
                            latestMatrix));
        }

        // HashMap<AprilTagId, Map<Double, Transform2d>> posesMap = new HashMap<>();

        // // Matrix<N3, N1> latestMatrix = VecBuilder.fill(0, 0, 0);

        // for (int i = 0; i < cameras.length; i++) {

        //     // if (!cameras[i].QueueToInputs().isPresent()) {
        //     //     continue;
        //     // }

        //     if (cameras[i].getTagNum() == -1) {
        //         continue;
        //     }

        //     var inputs = cameras[i].QueueToInputsTypeShi();

        //     if (inputs.isEmpty()) {
        //         break;
        //     }

        //     for (int j = 0; j < inputs.get().length; j++) {
        //         var bill = inputs.get()[j];
        //         if (bill != null
        //                 && bill.botToTarget.getX() > 0.05
        //                 && bill.botToTarget.getY() > 0.05) {

        //             if (!posesMap.containsKey(bill.id)) {
        //                 posesMap.put(bill.id, Map.of(bill.timestamp, bill.botToTarget));
        //             } else {
        //                 posesMap.get(bill.id).put(bill.timestamp, bill.botToTarget);
        //             }
        //         }
        //     }
        // }
        // double avgX2 = 0;
        // double avgY2 = 0;
        // double avgRot2 = 0;
        // double avgTimestamp2 = 0;
        // double count2 = 0;
        // if (!posesMap.isEmpty()) {
        //     for (AprilTagId id : posesMap.keySet()) {
        //         var posesById = posesMap.get(id);
        //         for (Map.Entry<Double, Transform2d> timestampedPose : posesById.entrySet()) {
        //             avgTimestamp2 += timestampedPose.getKey();
        //             avgX2 += timestampedPose.getValue().getX();
        //             avgY2 += timestampedPose.getValue().getY();
        //             avgRot2 += timestampedPose.getValue().getRotation().getDegrees();
        //             count++;
        //         }
        //         tagPoseAcceptor.accept(
        //                 new VisionInput(
        //                         avgTimestamp2 / count2,
        //                         id,
        //                         new Transform2d(
        //                                 avgX2 / count2,
        //                                 avgY2 / count2,
        //                                 Rotation2d.fromDegrees(avgRot2 / count2))));
        //         avgX2 = 0;
        //         avgY2 = 0;
        //         avgRot2 = 0;
        //         avgTimestamp2 = 0;
        //         count2 = 0;
        //     }
        // }
    }

    // public Pose2d getMeasurementByTimestamp(double timestamp) {
    //     return swerve.getPoseByTimestamp(timestamp);
    // }

    public boolean shouldAddData(Pose2d visionPose, Pose2d drivePose) {
        double distance =
                Math.sqrt(
                        Math.pow(visionPose.getX() - drivePose.getX(), 2)
                                + Math.pow(visionPose.getY() - drivePose.getY(), 2));
        return (distance < 0.3) ? true : false;
    }
}
