package team3647.frc2024.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2024.constants.ShooterConstants;
import team3647.frc2024.subsystems.ShooterLeft;
import team3647.frc2024.subsystems.ShooterRight;
import team3647.lib.LinearRegression;

public class ShooterCommands {

    public Command openLoop(DoubleSupplier bill) {
        return Commands.run(
                () -> {
                    shooterRight.openLoop(bill.getAsDouble() * ShooterConstants.kRightRatio);
                    shooterLeft.openLoop(
                            MathUtil.clamp(bill.getAsDouble() * ShooterConstants.kLeftRatio, 0, 1));
                },
                shooterRight,
                shooterLeft);
    }

    public Command setVelocity(DoubleSupplier bill) {
        return Commands.run(
                () -> {
                    shooterRight.setVelocity(bill.getAsDouble() * ShooterConstants.kRightRatio);
                    shooterLeft.setVelocity(bill.getAsDouble() * ShooterConstants.kLeftRatio);
                },
                shooterRight,
                shooterLeft);
    }

    public Command setVelocity(DoubleSupplier bill, DoubleSupplier ratio) {
        return Commands.run(
                () -> {
                    shooterRight.setVelocity(bill.getAsDouble() * ratio.getAsDouble());
                    shooterLeft.setVelocity(bill.getAsDouble() * (2 - ratio.getAsDouble()));
                },
                shooterRight,
                shooterLeft);
    }

    public Command setVelocityIndep(DoubleSupplier left, DoubleSupplier right) {
        return Commands.run(
                () -> {
                    shooterRight.setVelocity(right.getAsDouble());
                    shooterLeft.setVelocity(left.getAsDouble());
                },
                shooterRight,
                shooterLeft);
    }

    public Command kill() {
        return Commands.run(
                () -> {
                    shooterRight.openLoop(0);
                    shooterLeft.openLoop(0);
                },
                shooterRight,
                shooterLeft);
    }

    public Command killRight() {
        return Commands.run(() -> shooterRight.openLoop(0), shooterRight);
    }

    public Command killLeft() {
        return Commands.run(() -> shooterLeft.openLoop(0), shooterLeft);
    }

    public Command characterize() {
        SlewRateLimiter filter = new SlewRateLimiter(0.1);
        Map<Double, Double> voltageVelocityMap = new HashMap<>();
        return Commands.runEnd(
                () -> {
                    double desiredVoltage = filter.calculate(5);
                    shooterLeft.setVoltage(desiredVoltage);
                    voltageVelocityMap.put(desiredVoltage, shooterLeft.getVelocity());
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
                    LinearRegression linReg = new LinearRegression(xArray, yArray);
                    // y = ax + b
                    var a = linReg.slope();
                    var b = linReg.intercept();
                    var kS = -b / a;
                    var kV = a;
                    // SmartDashboard.putNumber("shooter kS", kS);
                    // SmartDashboard.putNumber("shooter kV", kV);
                },
                shooterLeft);
    }

    private final Set<Subsystem> requirements;

    public ShooterCommands(ShooterRight shooterRight, ShooterLeft shooterLeft) {
        this.shooterRight = shooterRight;
        this.shooterLeft = shooterLeft;
        this.requirements = Set.of(shooterRight, shooterLeft);
    }

    public final ShooterRight shooterRight;
    public final ShooterLeft shooterLeft;
}
