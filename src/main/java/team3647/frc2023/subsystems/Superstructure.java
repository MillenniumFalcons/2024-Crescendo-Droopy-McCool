package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2023.commands.IntakeCommands;
import team3647.frc2023.commands.KickerCommands;
import team3647.frc2023.commands.PivotCommands;
import team3647.frc2023.commands.ShooterCommands;

public class Superstructure {

    private final DoubleSupplier pivotAngleSupplier;
    private final double stowAngle = 60;
    private boolean hasPiece = false;

    public Command feed() {
        return kickerCommands.kick();
    }

    public Command spinUp() {
        return shooterCommands.shoot(() -> 1);
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

    public Command shoot() {
        return Commands.parallel(
                prep(), spinUp(), Commands.sequence(Commands.waitSeconds(1), feed(), ejectPiece()));
    }

    public Command shootStow() {
        return shoot().andThen(Commands.waitSeconds(0.5)).andThen(stowFromShoot());
    }

    public Command prep() {
        return pivotCommands.setAngle(pivotAngleSupplier);
    }

    public Command stowFromShoot() {
        return Commands.parallel(pivotCommands.setAngle(() -> stowAngle), shooterCommands.kill())
                .until(() -> pivot.angleReached(stowAngle, 5));
    }

    public Command intake() {
        return intakeCommands.intake();
    }

    public Command stowIntake() {
        return Commands.parallel(intakeCommands.kill(), kickerCommands.kill());
    }

    public Superstructure(
            Intake intake,
            Kicker kicker,
            Shooter shooter,
            Pivot pivot,
            DoubleSupplier pivotAngleSupplier) {
        this.intake = intake;
        this.kicker = kicker;
        this.shooter = shooter;
        this.pivot = pivot;
        this.pivotAngleSupplier = pivotAngleSupplier;

        intakeCommands = new IntakeCommands(intake);
        kickerCommands = new KickerCommands(kicker);
        shooterCommands = new ShooterCommands(shooter);
        pivotCommands = new PivotCommands(pivot);
    }

    private final Intake intake;
    private final Kicker kicker;
    private final Shooter shooter;
    private final Pivot pivot;
    public final IntakeCommands intakeCommands;
    public final KickerCommands kickerCommands;
    public final ShooterCommands shooterCommands;
    public final PivotCommands pivotCommands;
}
