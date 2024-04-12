package team3647.frc2024.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
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
    public static final String zoom = "zoom";
    public static final String amp = "amp";

    public static final Transform3d robotToBackLeft =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-9.2193),
                            Units.inchesToMeters(8),
                            Units.inchesToMeters(8.7885 + 1.75)),
                    new Rotation3d(0, -Math.PI / 180 * 28.12, 0)
                            .rotateBy(new Rotation3d(0, 0, Math.PI * 5 / 6 + 0.2)));

    public static final Transform3d robotToBackRight =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-9.2193),
                            Units.inchesToMeters(-8),
                            Units.inchesToMeters(8.7885 + 1.75)),
                    new Rotation3d(0, -Math.PI / 180 * 28.12, 0)
                            .rotateBy(new Rotation3d(0, 0, -Math.PI * 5 / 6 - 0.2)));

    public static final Transform3d robotToLeft =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(0.6556),
                            Units.inchesToMeters(10.8914),
                            Units.inchesToMeters(18.6206 + 1.75)),
                    new Rotation3d(0, -Math.PI / 9, 0)
                            .rotateBy(new Rotation3d(0, 0, Math.PI / 2 - Math.PI / 9)));

    public static final Transform3d robotToRight =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(0.6556),
                            Units.inchesToMeters(-10.8914),
                            Units.inchesToMeters(18.6206 + 1.75)),
                    new Rotation3d(0, -Math.PI / 9, 0)
                            .rotateBy(new Rotation3d(0, 0, -Math.PI / 2 + Math.PI / 9)));

    public static final Transform3d robotToZoom =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-5.6929), 0, Units.inchesToMeters(11.0746 + 1.75)),
                    new Rotation3d(0, -Math.PI / 18, 0).rotateBy(new Rotation3d(0, 0, Math.PI)));

    public static final Transform3d robotToAmp =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-4.25),
                            0,
                            Units.inchesToMeters(22.1875 + 1.75 + 0.125)),
                    new Rotation3d(0, -Units.degreesToRadians(35), 0)
                            .rotateBy(new Rotation3d(0, 0, Math.PI)));

    public static final edu.wpi.first.math.Vector<N3> zoomStdDevs = VecBuilder.fill(0.02, 0.02, 5);

    public static final Pose3d randompose =
            new Pose3d(new Translation3d(2, 2, 0), new Rotation3d());
}
