package team3647.frc2024.util;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class NeuralDetectorLimelight implements NeuralDetector {
    private final NetworkTable table;
    private final String limelightIdentifier;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber gregTYSubscriber;
    private final IntegerSubscriber IDSubscriber;

    public NeuralDetectorLimelight(String limelight) {
        this.limelightIdentifier = limelight;
        table = NetworkTableInstance.getDefault().getTable(limelightIdentifier);
        txSubscriber =
                table.getDoubleTopic("tx")
                        .subscribe(
                                0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        tySubscriber =
                table.getDoubleTopic("ty")
                        .subscribe(
                                0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        gregTYSubscriber =
                table.getDoubleArrayTopic("tcornxy")
                        .subscribe(
                                new double[] {},
                                PubSubOption.keepDuplicates(true),
                                PubSubOption.sendAll(true));
        IDSubscriber =
                table.getIntegerTopic("classID")
                        .subscribe(
                                -1, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    }

    public double getTX() {
        return txSubscriber.get(0);
    }

    public double getTY() {
        return tySubscriber.get(0);
    }

    public double getGregTY() {
        var bill = gregTYSubscriber.get(new double[] {});
        if (bill.length == 0) {
            return 0;
        }
        return bill[1];
    }

    public double getGregTX() {
        var bill = gregTYSubscriber.get(new double[] {});
        if (bill.length == 0) {
            return 0;
        }
        return bill[0];
    }

    public double getID() {
        return IDSubscriber.get(-1);
    }

    public boolean hasTarget() {
        return !(getTX() == 0 || getTY() == 0);
    }

    // public Optional<Transform2d> robotRelativeCamToPiece() {
    //     if (getGregTX() == 0 || getGregTY() == 0) {
    //         //  || getID() == -1) {
    //         return Optional.empty();
    //     }

    //     var gregorianY = (120 - getGregTY()) / 120 * 25;
    //     var gregorianX = (0 - getGregTX()) / 0 * 0;

    //     var gregorianYaw = Math.sqrt(gregorianX * gregorianX + gregorianY * gregorianY);

    //     var gregorianRoll = Math.atan(gregorianY / gregorianX);

    //     var gregorianRotation =
    //             new Rotation3d(0, 0, gregorianYaw)
    //                     .rotateBy(new Rotation3d(gregorianRoll, 0, 0))
    //                     .rotateBy(VisionConstants.limelightRotation);

    //     var x = Math.tan(gregorianRotation.getX()) * VisionConstants.limelightHeight;
    //     var y = Math.tan(gregorianRotation.getZ()) * x;

    //     var transform =
    //             new Transform2d(
    //                     new Translation2d(x - Units.inchesToMeters(7), y -
    // Units.inchesToMeters(7)),
    //                     new Rotation2d(gregorianRotation.getX()));

    //     return Optional.of(transform);
    // }

    // public Optional<Pose2d> pieceCoordinate(Supplier<Pose2d> drivePose) {
    //     var greg = robotRelativeCamToPiece();
    //     // SmartDashboard.putBoolean("is present", robotRelativeCamToPiece().isPresent());
    //     if (greg.isPresent()) {
    //         var bill = greg.get();
    //         var pieceCoord =
    //                 drivePose
    //                         .get()
    //                         .transformBy(
    //                                 new Transform2d(
    //                                         new Translation2d(VisionConstants.limelightX, 0),
    //                                         new Rotation2d())) // robot to cam
    //                         .transformBy(bill);
    //         return Optional.of(pieceCoord);
    //     } else {
    //         return Optional.empty();
    //     }
    // }
}
