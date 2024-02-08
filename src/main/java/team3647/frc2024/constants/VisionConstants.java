package team3647.frc2024.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String photonIP_1 = "10.36.47.14";
    public static final String photonIP_2 = "10.36.47.14";
    public static final String photonIP_3 = "10.36.47.14";

    public static final String photonName_1_BackLeft = "ar doo cam";
    public static final String photonName_1_BackRight = "ar doo cam";
    public static final String photonName_1_Left = "ar doo cam";
    public static final String photonName_1_Right = "ar doo cam";
    public static final String photonName_1_Driver = "ar doo cam";

    public static final Transform3d robotToPhotonCam =
            new Transform3d(
                    new Translation3d(
                            -Units.inchesToMeters(11),
                            Units.inchesToMeters(11),
                            Units.inchesToMeters(7.5)),
                    new Rotation3d(Math.PI, 0, 0)
                            .rotateBy(new Rotation3d(0, -Math.PI / 6, 0))
                            .rotateBy(new Rotation3d(0, 0, Math.PI * 3 / 4)));
}
