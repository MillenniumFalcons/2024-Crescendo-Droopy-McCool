package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import team3647.frc2023.util.AprilTagPhotonVision;

public class VisionConstants {
    public static final String limelightName = "bill";
    public static final String limelightIP = "10.36.47.11";
    public static final double limelightX = Units.inchesToMeters(12.25);
    public static final double limelightHeight = Units.inchesToMeters(30.75);
    public static final double limelightPitch = Units.degreesToRadians(-30);
    public static final Rotation3d limelightRotation = new Rotation3d(0, limelightPitch, 0);
    public static final Transform3d limelightTransform =
            new Transform3d(new Translation3d(0, 0, limelightHeight), limelightRotation);
    public static final String photonName = "ar doo cam";
    public static final String photonIP = "10.36.47.14";
    public static final AprilTagPhotonVision ar_doo_cam =
            new AprilTagPhotonVision(
                    photonName,
                    new Transform3d(
                            new Translation3d(),
                            new Rotation3d(0, Units.degreesToRadians(-12), Math.PI)));
}
