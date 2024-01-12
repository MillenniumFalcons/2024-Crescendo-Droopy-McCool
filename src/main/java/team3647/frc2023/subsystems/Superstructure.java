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

    public Command feed() {
        return kickerCommands.kick();
    }

    public Command spinUp() {
        return shooterCommands.shoot(() -> 1);
    }

    public Command shoot() {
        return Commands.parallel(
                pivotCommands.setAngle(pivotAngleSupplier),
                spinUp(),
                Commands.sequence(Commands.waitSeconds(1), feed()));
    }

    public Command prep() {
        return pivotCommands.setAngle(pivotAngleSupplier);
    }

    public Command stow() {
        return Commands.parallel(
                pivotCommands.setAngle(() -> 90),
                kickerCommands.kill(),
                intakeCommands.kill(),
                shooterCommands.kill());
    }

    public Command intake() {
        return intakeCommands.intake();
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
