package team3647.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeomUtil {
    public static double distance(Pose2d pose1, Pose2d pose2) {
        final var pose = pose1.minus(pose2);
        return Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2));
    }

    public static double distanceSquared(Pose2d pose1, Pose2d pose2) {
        final var pose = pose1.minus(pose2);
        return Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2);
    }

    public static double distance(Transform2d transform) {
        var distancSquared = Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2);
        var distance = Math.sqrt(distancSquared);
        return distance;
    }

    public static double distanceSquared(Transform2d transform) {
        var distancSquared = Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2);
        return distancSquared;
    }

    public static double distanceSquared(Translation2d transform) {
        var distancSquared = Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2);
        return distancSquared;
    }
}
