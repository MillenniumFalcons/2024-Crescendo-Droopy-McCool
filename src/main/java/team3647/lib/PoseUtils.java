package team3647.lib;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

public class PoseUtils {
    
    /**
     * 
     * @param radius In meters
     * @return if the pose is within the radius given
     */
    public static boolean boundingRadius(Pose2d measure, Pose2d target, double radius){
        return Math.abs(measure.getX() - target.getX()) < radius
            && Math.abs(measure.getY() - target.getY()) < radius;
    }

    public static boolean boundingTriangle(Pose2d measure, Pose2d one, Pose2d two, Pose2d three){
        return false;
    }

    /** 
     * |----+y+y|
     * |        x       |
     * |--------|      ---
     * @param measure
     * @param x
     * @param y
     * @return
     */
    public static boolean boundingBox(Pose2d measure, Pose2d target, double x, double y){
        return false;
    }
}
