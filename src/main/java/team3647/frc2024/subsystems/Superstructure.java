package team3647.frc2024.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2024.commands.IntakeCommands;
import team3647.frc2024.commands.KickerCommands;
import team3647.frc2024.commands.PivotCommands;
import team3647.frc2024.commands.ShooterCommands;
import team3647.frc2024.commands.WristCommands;
import team3647.frc2024.constants.ShooterConstants;

public class Superstructure {

    private final Intake intake;
    private final Kicker kicker;
    private final ShooterRight shooterRight;
    private final ShooterLeft shooterLeft;
    private final Pivot pivot;
    private final Wrist wrist;
    public final IntakeCommands intakeCommands;
    public final KickerCommands kickerCommands;
    public final ShooterCommands shooterCommands;
    public final PivotCommands pivotCommands;
    public final WristCommands wristCommands;

    private final DoubleSupplier pivotAngleSupplier;
    private final DoubleSupplier shooterSpeedSupplier;
    private final double pivotStowAngle = 40;
    private final double wristStowAngle = 100;
    private final double wristIntakeAngle = 0;
    private final double shootSpeed;
    private boolean hasPiece = false;
    private boolean isClimbing;

    private final BooleanSupplier swerveAimed;

    public Superstructure(
            Intake intake,
            Kicker kicker,
            ShooterRight shooterRight,
            ShooterLeft shooterLeft,
            Pivot pivot,
            Wrist wrist,
            DoubleSupplier pivotAngleSupplier,
            DoubleSupplier shooterSpeedSuppler,
            double shootSpeed,
            BooleanSupplier swerveAimed) {
        this.intake = intake;
        this.kicker = kicker;
        this.shooterRight = shooterRight;
        this.shooterLeft = shooterLeft;
        this.pivot = pivot;
        this.pivotAngleSupplier = pivotAngleSupplier;
        this.shooterSpeedSupplier = shooterSpeedSuppler;
        this.shootSpeed = shootSpeed;
        this.wrist = wrist;
        this.swerveAimed = swerveAimed;

        intakeCommands = new IntakeCommands(intake);
        kickerCommands = new KickerCommands(kicker);
        shooterCommands = new ShooterCommands(shooterRight, shooterLeft);
        pivotCommands = new PivotCommands(pivot);
        wristCommands = new WristCommands(wrist);
    }

    public Command feed() {
        return kickerCommands.kick();
    }

    public Command setIsClimbing() {
        return Commands.runOnce(() -> this.isClimbing = true);
    }

    public Command setIsNotClimbing() {
        return Commands.runOnce(() -> this.isClimbing = false);
    }

    public boolean isClimbing() {
        return isClimbing;
    }

    public Command spinUp() {
        return shooterCommands.setVelocity(
                () -> getDesiredSpeed(), () -> ShooterConstants.kLeftRatio);
    }

    public Command spinUpAmp() {
        return shooterCommands.setVelocity(
                () -> shooterSpeedSupplier.getAsDouble() * 2 / getDesiredSpeed());
    }

    public double getDesiredSpeed() {
        return shootSpeed;
    }

    public double getWantedPivot() {
        return pivotAngleSupplier.getAsDouble();
    }

    public boolean getPiece() {
        return hasPiece;
    }

    public boolean currentYes() {
        return intake.getMasterCurrent() > 38 && wrist.getAngle() < 5; // 41
    }

    public Command setPiece() {
        return Commands.runOnce(() -> this.hasPiece = true);
    }

    public Command ejectPiece() {
        return Commands.runOnce(() -> this.hasPiece = false);
    }

    public boolean flywheelReadY() {
        return shooterLeft.velocityReached(20, 1);
    }

    public boolean pivotReady() {
        return pivot.angleReached(pivotAngleSupplier.getAsDouble(), 3);
    }

    public boolean swerveReady() {
        return swerveAimed.getAsBoolean();
    }

    public Command shoot() {
        return Commands.parallel(
                prep(), spinUp()
                // Commands.sequence(
                //         // Commands.waitSeconds(2.5),
                //         Commands.waitUntil(
                //                         () ->
                //                                 shooterLeft.velocityReached(30, 2)
                //                                         && pivot.angleReached(
                //                                                 pivotAngleSupplier.getAsDouble(),
                // 5)
                //                                         && swerveAimed.getAsBoolean())
                //                 .withTimeout(1.2),
                //         feed())
                );
    }

    public boolean aimedAtSpeaker() {
        return shooterLeft.velocityReached(30, 1)
                && pivot.angleReached(pivotAngleSupplier.getAsDouble(), 5)
                && swerveAimed.getAsBoolean();
    }

    public Command shootAmp() {
        return Commands.parallel(
                prepAmp(),
                spinUpAmp(),
                Commands.sequence(
                        // Commands.waitSeconds(2.5),
                        Commands.waitUntil(
                                        () ->
                                                shooterLeft.velocityReached(
                                                                shootSpeed
                                                                        * 2
                                                                        / getDesiredSpeed()
                                                                        * ShooterConstants
                                                                                .kLeftRatio,
                                                                1)
                                                        && pivot.angleReached(55, 5)
                                                        && swerveAimed.getAsBoolean())
                                .withTimeout(0.7),
                        feed()));
    }

    public Command batterShot() {
        return Commands.parallel(
                batterPrep(),
                spinUp(),
                Commands.sequence(
                        // Commands.waitSeconds(2.5),
                        Commands.waitUntil(
                                        () ->
                                                shooterLeft.velocityReached(30, 1)
                                                        && pivot.angleReached(60, 5))
                                .withTimeout(0.5),
                        feed()));
    }

    public Command shootManual() {
        return Commands.parallel(
                prepManual(),
                spinUp(),
                Commands.sequence(
                        Commands.waitSeconds(2), Commands.parallel(intake(), ejectPiece())));
    }

    public Command shootStow() {
        return shoot().andThen(Commands.waitSeconds(0.5)).andThen(stowFromShoot());
    }

    public Command prep() {
        return pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble());
    }

    public Command prepAmp() {
        return pivotCommands.setAngle(() -> 35);
    }

    public Command batterPrep() {
        return pivotCommands.setAngle(() -> 60);
    }

    public Command prepManual() {
        return pivotCommands.setAngle(() -> SmartDashboard.getNumber("pivot interp angle", 40));
    }

    public Command stowFromShoot() {
        return Commands.sequence(
                Commands.parallel(prep(), spinUp(), feed()).withTimeout(1),
                Commands.parallel(
                                pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble()),
                                shooterCommands.kill(),
                                kickerCommands.kill())
                        .withTimeout(1));
    }

    public Command intake() {
        return Commands.parallel(wristCommands.setAngle(wristIntakeAngle), intakeCommands.intake());
    }

    public Command passToShooter() {
        return Commands.parallel(
                        intakeCommands.kill(),
                        kickerCommands.kick(),
                        wristCommands.setAngle(() -> 110).until(() -> wrist.angleReached(110, 5)))
                .withTimeout(0.3)
                .andThen(shootThrough());
    }

    public Command passToShooterNoKicker(Trigger shouldGO) {
        return Commands.parallel(
                        intakeCommands.kill(),
                        wristCommands.setAngle(() -> wrist.getInverseKinematics(pivot.getAngle())))
                .until(shouldGO)
                .andThen(shootThroughNoKicker());
    }

    public Command shootThrough() {
        return Commands.parallel(intakeCommands.intake(), kickerCommands.kick())
                .until(() -> pivot.frontPiece())
                .andThen(slightReverse().withTimeout(0.1))
                // .withTimeout(1)
                .andThen(stowIntake());
    }

    public Command shootThroughNoKicker() {
        return intakeCommands
                .intake()
                .until(() -> pivot.backPiece())
                .withTimeout(0.6)
                .andThen(stowIntake());
    }

    public Command stowIntake() {
        return Commands.parallel(
                wristCommands
                        .setAngle(wristStowAngle)
                        .until(() -> wrist.angleReached(wristStowAngle, 5)),
                intakeCommands.kill());
    }

    public boolean wristAtStow() {
        return wrist.angleReached(wristStowAngle, 5);
    }

    public Command autoFeed(BooleanSupplier goodToGo) {
        return Commands.run(
                () -> {
                    if (goodToGo.getAsBoolean() && pivot.backPiece()) {
                        Commands.sequence(
                                Commands.waitUntil(
                                        () ->
                                                pivot.angleReached(
                                                        pivotAngleSupplier.getAsDouble(), 2)),
                                kickerCommands.fastKick());
                    } else if (!frontPiece() && !pivot.backPiece()) {
                        slightForwards();
                    } else {
                        kickerCommands.kill();
                    }
                });
    }

    public Command fastFeed() {
        return kickerCommands.fastKick();
    }

    public Command index() {
        if (pivot.frontPiece()) {
            return slightReverse().until(() -> !pivot.frontPiece());
        } else {
            return slightForwards().until(() -> pivot.frontPiece());
        }
    }

    public Command stowIntakeAndIndex() {
        return Commands.parallel(stowIntake(), index()).withTimeout(0.2);
    }

    public boolean frontPiece() {
        return pivot.frontPiece();
    }

    public Command slightForwards() {
        return kickerCommands.slowFeed();
    }

    public Command slightReverse() {
        return kickerCommands.unkick();
    }
}
