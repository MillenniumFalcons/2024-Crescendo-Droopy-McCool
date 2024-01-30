package team3647.frc2023.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Shooter;
import team3647.lib.LinearRegression;

public class ShooterCommands {

    public Command shoot(DoubleSupplier bill) {
        return Commands.run(() -> shooter.openLoop(bill.getAsDouble()), shooter);
    }

    public Command setVelocity(DoubleSupplier bill) {
        return Commands.run(() -> shooter.setVelocity(bill.getAsDouble()), shooter);
    }

    public Command kill() {
        return Commands.run(() -> shooter.openLoop(0), shooter);
    }

    public Command characterize() {
        SlewRateLimiter filter = new SlewRateLimiter(0.1);
        Map<Double, Double> voltageVelocityMap = new HashMap<>();
        return Commands.runEnd(
                () -> {
                    double desiredVoltage = filter.calculate(5);
                    shooter.setVoltage(desiredVoltage);
                    voltageVelocityMap.put(desiredVoltage, shooter.getVelocity());
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
                    SmartDashboard.putNumber("shooter kS", kS);
                    SmartDashboard.putNumber("shooter kV", kV);
                },
                shooter);
    }

    private final Set<Subsystem> requirements;

    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
        this.requirements = Set.of(shooter);
    }

    public final Shooter shooter;
}
