// package team3647.frc2024.util;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.DoubleArraySubscriber;
// import edu.wpi.first.networktables.IntegerSubscriber;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.PubSubOption;
// import edu.wpi.first.networktables.TimestampedDoubleArray;
// import java.util.Optional;
// import team3647.lib.vision.AprilTagCamera.AprilTagId;

// public class AprilTagLimelight implements AprilTagCamera {
//     private final NetworkTable table;
//     private final NetworkTableEntry botPoseEntry;
//     private final String limelightIdentifier;
//     private final DoubleArraySubscriber botPoseSubscriber;
//     private final DoubleArraySubscriber botToTargetSubscriber;
//     private final IntegerSubscriber tagNumberSubscriber;
//     private final double stdDevs = 0.01;

//     public AprilTagLimelight(String limelight) {
//         this.limelightIdentifier = limelight;
//         table = NetworkTableInstance.getDefault().getTable(limelightIdentifier);
//         botPoseEntry = table.getEntry("botpose_wpiblue");
//         tagNumberSubscriber = table.getIntegerTopic("tid").subscribe(-1);
//         botToTargetSubscriber =
//                 table.getDoubleArrayTopic("botpose_targetspace")
//                         .subscribe(
//                                 new double[] {},
//                                 PubSubOption.keepDuplicates(true),
//                                 PubSubOption.sendAll(true));
//         botPoseSubscriber =
//                 table.getDoubleArrayTopic("botpose_wpiblue")
//                         .subscribe(
//                                 new double[] {},
//                                 PubSubOption.keepDuplicates(true),
//                                 PubSubOption.sendAll(true));
//     }

//     public DoubleArraySubscriber getBotPoseSubscriber() {
//         return botPoseSubscriber;
//     }

//     public DoubleArraySubscriber getBotToTargetSubscriber() {
//         return botToTargetSubscriber;
//     }

//     public IntegerSubscriber getTagNumSubscriber() {
//         return tagNumberSubscriber;
//     }

//     public int getTagNum() {
//         return ((int) tagNumberSubscriber.get(-1));
//     }

//     public static AprilTagId getId(int id) {
//         var adjustedId = id - 1;
//         var possibleValues = AprilTagId.values();
//         if (adjustedId < 0 || adjustedId > possibleValues.length) {
//             return AprilTagId.ID_DNE;
//         }

//         return possibleValues[adjustedId];
//     }

//     public Optional<VisionMeasurement[]> QueueToInputs() {
//         var botPoseQueue = this.getBotPoseSubscriber().readQueue();
//         var botToTargetQueue = this.getBotToTargetSubscriber().readQueue();
//         if (botPoseQueue.length < 1 || botToTargetQueue.length < 1) {
//             return Optional.empty();
//         }
//         var inputs = new VisionMeasurement[botPoseQueue.length];
//         var length =
//                 botPoseQueue.length <= botToTargetQueue.length
//                         ? botPoseQueue.length
//                         : botToTargetQueue.length;
//         for (TimestampedDoubleArray botPose : botPoseQueue) {}

//         for (int i = 0; i < length; i++) {
//             var botPose = botPoseQueue[i].value;
//             var timestamp = botPoseQueue[i].timestamp;
//             var botToTag = botToTargetQueue[i].value;
//             Pose2d tagDistance = new Pose2d(botToTag[0], botToTag[1], new Rotation2d());
//             double distanceSquared =
//                     Math.pow(tagDistance.getX(), 2) + Math.pow(tagDistance.getY(), 2);
//             double stdDev = stdDevs * distanceSquared;
//             inputs[i] =
//                     new VisionMeasurement(
//                             new Pose3d(
//                                             botPose[0],
//                                             botPose[1],
//                                             botPose[2],
//                                             new Rotation3d(
//                                                     Units.degreesToRadians(botPose[3]),
//                                                     Units.degreesToRadians(botPose[4]),
//                                                     Units.degreesToRadians(botPose[5])))
//                                     .toPose2d(),
//                             (timestamp - (botPose[6] / 1000.0)) / 1000000,
//                             VecBuilder.fill(stdDev, stdDev, stdDev));
//         }
//         return Optional.of(inputs);
//     }
// }
