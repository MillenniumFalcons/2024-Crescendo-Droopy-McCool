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
import team3647.lib.LinearRegression;

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
                    boolean openloop = true;
                    var motionXComponent = ySpeedFunction.getAsDouble() * maxSpeed * triggerSlow;
                    // right stick X, (negative so that left positive)
                    var motionYComponent = -xSpeedFunction.getAsDouble() * maxSpeed * triggerSlow;

                    var motionTurnComponent =
                            -turnSpeedFunction.getAsDouble() * maxRotationRadpS * triggerSlow;

                    // var translation = new Translation2d(motionXComponent, motionYComponent);

                    if (mode == DriveMode.SHOOT_ON_THE_MOVE && enabeld) {
                        motionTurnComponent = autoDriveTwist2d.dtheta;
                    }

                    var translation = new Translation2d(motionXComponent, motionYComponent);

                    var rotation = motionTurnComponent;
                    swerve.driveFieldOriented(translation.getX(), translation.getY(), rotation);
                },
                swerve);
    }

    public Command characterize() {
        SlewRateLimiter filter = new SlewRateLimiter(0.2);
        Map<Double, Double> voltageVelocityMap = new HashMap<>();
        Map<Double, Double> voltageAccelMap = new HashMap<>();
        return Commands.runEnd(
                () -> {
                    double desiredVoltage = filter.calculate(12);

                    swerve.drive(desiredVoltage, 0, 0);
                    voltageVelocityMap.put(
                            desiredVoltage, swerve.getChassisSpeeds().vxMetersPerSecond);
                    voltageAccelMap.put(desiredVoltage, swerve.getAccel());
                    SmartDashboard.putNumber("voltage", desiredVoltage);
                    SmartDashboard.putNumber(
                            "velocity", swerve.getChassisSpeeds().vxMetersPerSecond);
                },
                () -> {
                    var xArray =
                            voltageVelocityMap.keySet().stream()
                                    .mapToDouble(Double::doubleValue)
                                    .toArray();
                    var yArray =
                            voltageVelocityMap.values().stream()
                                    .mapToDouble(Double::doubleValue)
                                    .toArray();
                    var xArray2 =
                            voltageAccelMap.keySet().stream()
                                    .mapToDouble(Double::doubleValue)
                                    .toArray();
                    var yArray2 =
                            voltageAccelMap.values().stream()
                                    .mapToDouble(Double::doubleValue)
                                    .toArray();
                    LinearRegression linReg = new LinearRegression(xArray, yArray);
                    LinearRegression linReg2 = new LinearRegression(xArray2, yArray2);

                    // y = ax + b
                    var a = linReg.slope();
                    var a2 = linReg2.slope();
                    var b = linReg.intercept();
                    var kS = -b / a;
                    var kV = a;
                    var kA = a2;
                    SmartDashboard.putNumber("dt kS", kS);
                    SmartDashboard.putNumber("dt kV", kV);
                    SmartDashboard.putNumber("dt kA", kA);
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
