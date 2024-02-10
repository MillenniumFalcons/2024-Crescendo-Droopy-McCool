package team3647.frc2024.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.BooleanSupplier;

public class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxRotSpeedRadPerSec =
            SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec;

    public static final double kMaxAccelerationMetersPerSecSq = 3;
    public static final double kMaxRotAccelerationRadsPerSecSq = Math.PI;

    public static final BooleanSupplier isRed;

    public static final PathConstraints defaultConstraints =
            new PathConstraints(
                    kMaxRotSpeedRadPerSec,
                    kMaxAccelerationMetersPerSecSq,
                    kMaxRotSpeedRadPerSec,
                    kMaxRotAccelerationRadsPerSecSq);

    public static final double kXControllerP = 5; // 1.4, 2.2;
    public static final double kXControllerI = 0; // 6;
    public static final double kXControllerD = 0; // 0;

    public static final double kYControllerP = 5; // 1.4, 2.2;
    public static final double kYControllerI = 0; // 6;
    public static final double kYControllerD = 0; // 0;

    public static final double xRotControllerP = 5; // 12;
    public static final double xRotControllerI = 0; // 8;
    public static final double xRotControllerD = 0.8;

    public static final TrapezoidProfile.Constraints kRotControllerConstraints =
            new TrapezoidProfile.Constraints(
                    kMaxRotSpeedRadPerSec, kMaxRotAccelerationRadsPerSecSq);

    public static final PIDConstants kTranslationConstants =
            new PIDConstants(kXControllerP, kXControllerI, kXControllerD);
    public static final PIDConstants kRotationConstants =
            new PIDConstants(xRotControllerP, xRotControllerI, xRotControllerD);

    public static final PIDController kXController =
            new PIDController(kXControllerP, kXControllerI, kXControllerD);
    public static final PIDController kYController =
            new PIDController(kYControllerP, kYControllerI, kYControllerD);
    public static final PIDController kRotController =
            new PIDController(xRotControllerP, xRotControllerI, xRotControllerD);

    public static final double kDrivetrainXShootingThreshold = 4.5;

    static {
        kRotController.enableContinuousInput(-Math.PI, Math.PI);

        isRed =
                () -> {
                    return DriverStation.getAlliance().get() == Alliance.Red;
                };
    }

    private AutoConstants() {}
}
