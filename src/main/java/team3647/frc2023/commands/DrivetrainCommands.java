package team3647.frc2023.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.util.AutoDrive.DriveMode;

public class DrivetrainCommands {

    public Command driveVisionTeleop(
            DoubleSupplier xSpeedFunction, // X axis on joystick is Left/Right
            DoubleSupplier ySpeedFunction, // Y axis on Joystick is Front/Back
            DoubleSupplier turnSpeedFunction,
            BooleanSupplier slowTriggerFunction,
            // BooleanSupplier enableAutoSteer,
            BooleanSupplier getIsFieldOriented,
            Supplier<DriveMode> autoDriveMode,
            BooleanSupplier autoDriveEnabled,
            Supplier<Twist2d> autoDriveVelocities
            // Supplier<Twist2d> autoSteerVelocitiesSupplier
            ) {
        return Commands.run(
                () -> {
                    boolean enabeld = autoDriveEnabled.getAsBoolean();
                    DriveMode mode = autoDriveMode.get();
                    Twist2d autoDriveTwist2d = autoDriveVelocities.get();
                    double triggerSlow = slowTriggerFunction.getAsBoolean() ? 0.6 : 1;
                    // boolean autoSteer = enableAutoSteer.getAsBoolean();
                    boolean fieldOriented = getIsFieldOriented.getAsBoolean();
                    boolean openloop = false;
                    var motionXComponent = ySpeedFunction.getAsDouble() * maxSpeed * triggerSlow;
                    // right stick X, (negative so that left positive)
                    var motionYComponent = -xSpeedFunction.getAsDouble() * maxSpeed * triggerSlow;

                    var motionTurnComponent =
                            -turnSpeedFunction.getAsDouble() * maxRotationRadpS * triggerSlow;

                    // var translation = new Translation2d(motionXComponent, motionYComponent);

                    if (mode == DriveMode.SHOOT_STATIONARY && enabeld) {
                        motionTurnComponent = autoDriveTwist2d.dtheta;
                        motionXComponent = 0;
                        motionYComponent = 0;
                    }

                    if (mode == DriveMode.SHOOT_ON_THE_MOVE && enabeld) {
                        motionTurnComponent = autoDriveTwist2d.dtheta;
                    }

                    var translation = new Translation2d(motionXComponent, motionYComponent);

                    var rotation = motionTurnComponent;
                    swerve.drive(translation, rotation, fieldOriented, openloop);
                },
                swerve);
    }

    public Command characterize() {
        SlewRateLimiter filter = new SlewRateLimiter(0.01);
        Map<Double, Double> voltageVelocityMap = new HashMap<>();
        return Commands.runEnd(
                () -> {
                    double desiredVoltage = filter.calculate(12);
                    swerve.setCharacterizationVoltage(desiredVoltage);
                    voltageVelocityMap.put(
                            desiredVoltage, swerve.getChassisSpeeds().vxMetersPerSecond);
                },
                () -> {
                    var xArray = voltageVelocityMap.keySet().stream().toArray(Double[]::new);
                    var yArray = voltageVelocityMap.entrySet().stream().toArray(Double[]::new);
                    var xSum = 0;
                    var ySum = 0;
                    var xSquareSum = 0;
                    var xySum = 0;
                    for (int i = 0; i < xArray.length; i++) {
                        double x = xArray[i];
                        double y = yArray[i];
                        xSum += x;
                        xSquareSum += x * x;
                        ySum += y;
                        xySum += x * y;
                    }
                    // y = a + bx
                    var a =
                            (ySum * xSquareSum - xSum * xySum)
                                    / (xArray.length * xSquareSum - xSum * xSum);
                    var b =
                            (xArray.length * xySum - xSum * ySum)
                                    / (xArray.length * xSquareSum - xSum * xSum);
                    var kS = -a / b;
                    var kA = 1 / b;
                    SmartDashboard.putNumber("drivetrain kS", kS);
                    SmartDashboard.putNumber("drivetrain kA", kA);
                },
                swerve);
    }

    private final SwerveDrive swerve;
    private static final Rotation2d OneEightyRotation = FieldConstants.kOneEighty;
    private final double maxSpeed;
    private final double maxRotationRadpS;

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
        this.maxSpeed = this.swerve.getMaxSpeedMpS();
        this.maxRotationRadpS = this.swerve.getMaxRotationRadpS();
    }
}
