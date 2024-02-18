package team3647.frc2024.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2024.commands.IntakeCommands;
import team3647.frc2024.commands.KickerCommands;
import team3647.frc2024.commands.PivotCommands;
import team3647.frc2024.commands.ShooterCommands;
import team3647.frc2024.commands.WristCommands;

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
    private final double pivotStowAngle = 40;
    private final double wristStowAngle = 100;
    private final double wristIntakeAngle = 0;
    private final double shootSpeed;
    private boolean hasPiece = false;

    private final DoubleSupplier wristInverseKinematics;

    private final BooleanSupplier swerveAimed;

    public Superstructure(
            Intake intake,
            Kicker kicker,
            ShooterRight shooterRight,
            ShooterLeft shooterLeft,
            Pivot pivot,
            Wrist wrist,
            DoubleSupplier pivotAngleSupplier,
            double shootSpeed,
            DoubleSupplier wristInverseKinematics,
            BooleanSupplier swerveAimed) {
        this.intake = intake;
        this.kicker = kicker;
        this.shooterRight = shooterRight;
        this.shooterLeft = shooterLeft;
        this.pivot = pivot;
        this.pivotAngleSupplier = pivotAngleSupplier;
        this.shootSpeed = shootSpeed;
        this.wrist = wrist;
        this.wristInverseKinematics = wristInverseKinematics;
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

    public Command spinUp() {
        return shooterCommands.setVelocity(() -> shootSpeed);
    }

    public double getDesiredSpeed() {
        return shootSpeed;
    }

    public double getInverseKinematics() {
        return wristInverseKinematics.getAsDouble();
    }

    public double getWantedPivot() {
        return pivotAngleSupplier.getAsDouble();
    }

    public boolean getPiece() {
        return hasPiece;
    }

    public boolean currentYes() {
        return intake.getMasterCurrent() > 32 && wrist.getAngle() < 5;
    }

    public Command setPiece() {
        return Commands.runOnce(() -> this.hasPiece = true);
    }

    public Command ejectPiece() {
        return Commands.runOnce(() -> this.hasPiece = false);
    }

    public Command shoot() {
        return Commands.parallel(
                prep(),
                spinUp(),
                Commands.sequence(
                        // Commands.waitSeconds(3),
                        Commands.waitUntil(
                                        () ->
                                                shooterLeft.velocityReached(shootSpeed * 1.1, 1)
                                                        && pivot.angleReached(
                                                                pivotAngleSupplier.getAsDouble(), 3)
                                                        && swerveAimed.getAsBoolean())
                                .withTimeout(2),
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
        // SmartDashboard.putNumber("pivot supplier", pivotAngleSupplier.getAsDouble());
        // return pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble());
        return pivotCommands.setAngle(() -> SmartDashboard.getNumber("pivot interp", 35));
    }

    public Command prepManual() {
        return pivotCommands.setAngle(() -> SmartDashboard.getNumber("pivot interp angle", 40));
    }

    public Command stowFromShoot() {
        return Commands.parallel(
                        pivotCommands.setAngle(() -> pivotStowAngle),
                        shooterCommands.kill(),
                        kickerCommands.kill())
                .until(() -> pivot.angleReached(pivotStowAngle, 5));
    }

    public Command intake() {
        return Commands.parallel(wristCommands.setAngle(wristIntakeAngle), intakeCommands.intake());
    }

    public Command passToShooter() {
        return Commands.parallel(
                        intakeCommands.kill(),
                        kickerCommands.kick(),
                        wristCommands
                                .setAngle(() -> wristInverseKinematics.getAsDouble())
                                .until(() -> wrist.angleReached(this.getInverseKinematics(), 5)))
                .withTimeout(0.5)
                .andThen(shootThrough());
    }

    public Command passToShooterNoKicker() {
        return Commands.parallel(
                        intakeCommands.kill(),
                        wristCommands.setAngle(() -> this.getInverseKinematics()))
                .withTimeout(0.5)
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
                .withTimeout(0.5)
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
                                feed());
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
