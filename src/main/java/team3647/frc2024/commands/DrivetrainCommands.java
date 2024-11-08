package team3647.frc2024.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2024.constants.FieldConstants;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.frc2024.util.AutoDrive.DriveMode;

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
                    int invert = 1;
                    if (DriverStation.getAlliance().isPresent()) {
                        if (DriverStation.getAlliance().get() == Alliance.Red) {
                            invert = -1;
                        }
                    }
                    boolean enabeld = autoDriveEnabled.getAsBoolean();
                    DriveMode mode = autoDriveMode.get();
                    Twist2d autoDriveTwist2d = autoDriveVelocities.get();
                    double triggerSlow = slowTriggerFunction.getAsBoolean() ? 0.6 : 1;
                    // boolean autoSteer = enableAutoSteer.getAsBoolean();
                    boolean fieldOriented = getIsFieldOriented.getAsBoolean();
                    boolean openloop = true;
                    double y =
                            Math.pow(ySpeedFunction.getAsDouble(), 2)
                                    * Math.signum(ySpeedFunction.getAsDouble())
                                    * 1.05;
                    double x =
                            Math.pow(xSpeedFunction.getAsDouble(), 2)
                                    * Math.signum(xSpeedFunction.getAsDouble())
                                    * 1.05;
                    var motionXComponent = y * maxSpeed * triggerSlow * invert;
                    // right stick X, (negative so that left positive)
                    var motionYComponent = -x * maxSpeed * triggerSlow * invert;

                    var motionTurnComponent =
                            -turnSpeedFunction.getAsDouble() * maxRotationRadpS * triggerSlow;

                    if (mode == DriveMode.SHOOT_AT_AMP && enabeld) {
                        motionXComponent = autoDriveTwist2d.dx + motionXComponent * 0.1;
                        motionTurnComponent = autoDriveTwist2d.dtheta + motionTurnComponent * 0.1;

                        var translation = new Translation2d(motionXComponent, motionYComponent);

                        var rotation = motionTurnComponent;
                        swerve.driveFieldOriented(translation.getX(), translation.getY(), rotation);
                    } else if (mode == DriveMode.INTAKE_IN_AUTO && enabeld) {
                        motionXComponent = autoDriveTwist2d.dx;
                        motionYComponent = autoDriveTwist2d.dy;

                        var translation = new Translation2d(motionXComponent, motionYComponent);

                        var rotation = motionTurnComponent;
                        swerve.drive(translation.getX(), translation.getY(), rotation);
                    } else if (mode == DriveMode.FEED && enabeld) {
                        motionTurnComponent = autoDriveTwist2d.dtheta + motionTurnComponent;
                        swerve.driveFieldOriented(motionXComponent, motionYComponent, motionTurnComponent);
                    } else {

                        if (mode != DriveMode.NONE && enabeld) {
                            motionTurnComponent = autoDriveTwist2d.dtheta;
                        }

                        // SmartDashboard.putNumber("theta", autoDriveTwist2d.dtheta);

                        var translation = new Translation2d(motionXComponent, motionYComponent);

                        var rotation = motionTurnComponent;
                        swerve.driveFieldOriented(translation.getX(), translation.getY(), rotation);
                    }
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
