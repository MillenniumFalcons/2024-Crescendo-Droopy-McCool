package team3647.frc2024.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String photonIP_back = "10.36.47.14"; // has left and back left
    public static final String photonIP_left =
            "10.36.47.15"; // has driver and temp left and temp back left
    public static final String photonIP_right = "10.36.47.16"; // has right and back right

    public static final String backLeft = "back left";
    public static final String backRight = "back right";
    public static final String left = "left";
    public static final String right = "right";
    public static final String driver = "driver";

    public static final Transform3d robotToBackLeft =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-9.2193),
                            Units.inchesToMeters(8),
                            Units.inchesToMeters(8.7885)),
                    new Rotation3d(0, -Math.PI / 180 * 28.12, 0)
                            .rotateBy(new Rotation3d(0, 0, Math.PI * 5 / 6 + 0.2)));

    public static final Transform3d robotToBackRight =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-9.2193),
                            Units.inchesToMeters(-8),
                            Units.inchesToMeters(8.7885)),
                    new Rotation3d(0, -Math.PI / 180 * 28.12, 0)
                            .rotateBy(new Rotation3d(0, 0, -Math.PI * 5 / 6 - 0.2)));

    public static final Transform3d robotToLeft =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(1.4481),
                            Units.inchesToMeters(11.0043),
                            Units.inchesToMeters(18.4375)),
                    new Rotation3d(0, -Math.PI / 9, 0).rotateBy(new Rotation3d(0, 0, Math.PI / 2)));

    public static final Transform3d robotToRight =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-1.4481),
                            Units.inchesToMeters(-11.0043),
                            Units.inchesToMeters(18.4375)),
                    new Rotation3d(0, -Math.PI / 9, 0)
                            .rotateBy(new Rotation3d(0, 0, -Math.PI / 2)));
}
