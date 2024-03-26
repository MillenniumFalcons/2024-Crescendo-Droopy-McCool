// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2024.constants.ChurroConstants;
import team3647.frc2024.subsystems.Churro;

/** Add your docs here. */
public class ChurroCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> churro.setOpenloop(demand.getAsDouble()), churro);
    }

    public Command setAngle(double angle) {
        return Commands.run(() -> churro.setAngle(angle), churro);
    }

    public Command setAngleSpringy(double angle) {
        return Commands.run(() -> churro.setAngleSpringy(angle), churro);
    }

    public Command setTorque(double torque) {
        return Commands.run(() -> churro.setTorque(torque), churro);
    }

    public Command setAngle(DoubleSupplier angle) {
        return Commands.run(() -> churro.setAngle(angle.getAsDouble()), churro);
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = ChurroConstants.kInitialDegree;

            @Override
            public void initialize() {
                degreeAtStart = churro.getAngle();
            }

            @Override
            public void execute() {
                churro.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    private final Churro churro;
    private final Set<Subsystem> requirements;

    public ChurroCommands(Churro churro) {
        this.churro = churro;
        this.requirements = Set.of(churro);
    }
}
