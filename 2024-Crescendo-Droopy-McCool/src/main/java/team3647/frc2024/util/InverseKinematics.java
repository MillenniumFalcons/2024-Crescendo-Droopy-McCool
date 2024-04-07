package team3647.frc2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team3647.lib.GeomUtil;

public class InverseKinematics {

    private static final double defaultAngle = 0;
    private static final double wristAngleoffSet = Math.toRadians(34);
    private static final double pivotLength = Units.inchesToMeters(7.9854);
    private static final double pivotHeight = Units.inchesToMeters(9);
    private static final double pivotX = Units.inchesToMeters(9.75);
    private static final double wristHeight = 0;
    private static final double wristX = 0;
    private static final double wristLength = Units.inchesToMeters(11.2);
    private static final double wristRollersAngle = Math.toRadians(28);
    private static final Pose2d wristOriginPos = new Pose2d(wristX, wristHeight, new Rotation2d());
    private static final double interestingOffset = Math.toRadians(5);

    public InverseKinematics() {}

    // all poses are from the side pov with the intake at the back, positive x is forward and
    // positive y is up, 0 rot is forward and positive is ccw

    public static Pose2d getPivotReceivingPosition(double pivot) {
        return new Pose2d(
                pivotX - pivotLength * Math.cos(Math.toRadians(pivot + interestingOffset)),
                pivotHeight - pivotLength * Math.sin(Math.toRadians(pivot + interestingOffset)),
                new Rotation2d());
    }

    public static double getWristHandoffAngleByPivot(double pivot) {
        Pose2d pivotReceivingPose = getPivotReceivingPosition(pivot);
        double triangleBaseSqaured =
                GeomUtil.distanceSquared(
                        wristOriginPos,
                        pivotReceivingPose); // creating triangle with wrist length, wrist origin to
        // pivot receoving, and note to pivot receiving as
        // sides
        double triangleSide1 = wristLength;
        double quadraticA = 1; // law of cos
        double quadraticB = -2 * triangleSide1 * Math.cos(wristRollersAngle);
        double quadraticC = triangleSide1 * triangleSide1 - triangleBaseSqaured;
        if (quadraticB * quadraticB - 4 * quadraticA * quadraticC < 0) {
            return defaultAngle;
        }
        double triangleSide2 =
                (-quadraticB + Math.sqrt(quadraticB * quadraticB - 4 * quadraticA * quadraticC))
                        / (2 * quadraticA); // pick greater solution
        Pose2d intersection =
                getCircleIntersection(
                        wristOriginPos, triangleSide1, pivotReceivingPose, triangleSide2);
        double wristAngle =
                Math.atan(
                        (intersection.getY() - wristOriginPos.getY())
                                / (wristOriginPos.getX() - intersection.getX()));
        if (wristAngle < 0) { // account for domain
            wristAngle += Math.PI;
        }
        wristAngle += wristAngleoffSet; // difference between measured angle and angle to rollers;
        return Math.toDegrees(wristAngle) + 5;
    }

    public static Pose2d getCircleIntersection(Pose2d c1, double r1, Pose2d c2, double r2) {
        double distance = GeomUtil.distance(c1, c2);
        double d1 = (r1 * r1 - r2 * r2 + distance * distance) / (2 * distance);
        double h = Math.sqrt(r1 * r1 - d1 * d1);
        double x3 = c1.getX() + (d1 * (c2.getX() - c1.getX())) / distance;
        double y3 = c1.getY() + (d1 * (c2.getY() - c1.getY())) / distance;
        double x4 = x3 - (h * (c2.getY() - c1.getY())) / distance; // get the higher point
        double y4 = y3 + (h * (c2.getX() - c1.getX())) / distance;
        return new Pose2d(x4, y4, new Rotation2d());
    }
}
