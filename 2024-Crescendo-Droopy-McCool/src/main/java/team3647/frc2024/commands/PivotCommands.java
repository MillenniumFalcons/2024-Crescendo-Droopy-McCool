package team3647.frc2024.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2024.constants.PivotConstants;
import team3647.frc2024.subsystems.Pivot;

public class PivotCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> pivot.setOpenloop(0.15 * demand.getAsDouble()), this.pivot);
    }

    public Command setAngle(DoubleSupplier setpoint) {
        return Commands.run(() -> pivot.setAngle(setpoint.getAsDouble()), pivot);
    }

    public Command characterizeKG() {
        return Commands.run(
                () -> {
                    double voltage = SmartDashboard.getNumber("pivot characterization voltage", 0);
                    pivot.setVoltage(voltage);
                },
                pivot);
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = PivotConstants.kInitialAngle;

            @Override
            public void initialize() {
                degreeAtStart = pivot.getAngle();
            }

            @Override
            public void execute() {
                pivot.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    private final Pivot pivot;
    private final Set<Subsystem> requirements;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
        this.requirements = Set.of(pivot);
    }
}
