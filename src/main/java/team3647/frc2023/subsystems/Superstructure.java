package team3647.frc2023.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2023.commands.IntakeCommands;
import team3647.frc2023.commands.KickerCommands;
import team3647.frc2023.commands.PivotCommands;
import team3647.frc2023.commands.ShooterCommands;

public class Superstructure {

    private final Intake intake;
    private final Kicker kicker;
    private final Shooter shooter;
    private final Pivot pivot;
    public final IntakeCommands intakeCommands;
    public final KickerCommands kickerCommands;
    public final ShooterCommands shooterCommands;
    public final PivotCommands pivotCommands;

    private final DoubleSupplier pivotAngleSupplier;
    private final double stowAngle = 40;
    private final double shootSpeed;
    private boolean hasPiece = false;
    private boolean isShooting = false;

    public Superstructure(
            Intake intake,
            Kicker kicker,
            Shooter shooter,
            Pivot pivot,
            DoubleSupplier pivotAngleSupplier,
            double shootSpeed) {
        this.intake = intake;
        this.kicker = kicker;
        this.shooter = shooter;
        this.pivot = pivot;
        this.pivotAngleSupplier = pivotAngleSupplier;
        this.shootSpeed = shootSpeed;

        intakeCommands = new IntakeCommands(intake);
        kickerCommands = new KickerCommands(kicker);
        shooterCommands = new ShooterCommands(shooter);
        pivotCommands = new PivotCommands(pivot);
    }

    public Command feed() {
        return kickerCommands.kick();
    }

    public Command spinUp() {
        SlewRateLimiter filter = new SlewRateLimiter(3);
        return shooterCommands.setVelocity(() -> filter.calculate(shootSpeed));
    }

    public double getDesiredSpeed() {
        return shootSpeed;
    }

    public boolean getPiece() {
        return hasPiece;
    }

    public Command setPiece() {
        return Commands.runOnce(() -> this.hasPiece = true);
    }

    public Command ejectPiece() {
        return Commands.runOnce(() -> this.hasPiece = false);
    }

    public Command isSHooting() {
        return Commands.runOnce(() -> this.isShooting = true);
    }

    public Command doneShooting() {
        return Commands.runOnce(() -> this.isShooting = false);
    }

    public boolean getIsShooting() {
        return isShooting;
    }

    public Command shoot() {
        return Commands.parallel(
                        prep(),
                        spinUp(),
                        Commands.sequence(
                                Commands.waitSeconds(1),
                                // Commands.waitUntil(() -> shooter.velocityReached(shootSpeed, 1))
                                //         .withTimeout(3),
                                Commands.parallel(isSHooting(), intake(), ejectPiece())
                                        .withTimeout(1)))
                .finallyDo((interrupted) -> doneShooting());
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

    public Command outtake() {
        return Commands.parallel(intakeCommands.outtake(), kickerCommands.unkick());
    }

    public Command prep() {
        SmartDashboard.putNumber("pivot supplier", pivotAngleSupplier.getAsDouble());
        return pivotCommands.setAngle(() -> pivotAngleSupplier.getAsDouble());
    }

    public Command prepManual() {
        return pivotCommands.setAngle(() -> SmartDashboard.getNumber("pivot interp angle", 40));
    }

    public Command oscillateToCenter() {
        return Commands.parallel(intakeCommands.oscillate(), kickerCommands.oscillate());
    }

    public Command stowFromShoot() {
        return Commands.parallel(
                        pivotCommands.setAngle(() -> stowAngle),
                        shooterCommands.kill(),
                        kickerCommands.kill(),
                        intakeCommands.kill())
                .until(() -> pivot.angleReached(stowAngle, 5));
    }

    public Command intake() {
        return Commands.parallel(intakeCommands.intake(), kickerCommands.kick());
    }

    public Command stowIntake() {
        return Commands.parallel(intakeCommands.kill(), kickerCommands.kill());
    }

    public Command slightReverse() {
        return outtake().withTimeout(0.4).andThen(stowIntake());
    }
}
