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

public class ShooterCommands {

    public Command shoot(DoubleSupplier bill) {
        return Commands.run(() -> shooter.openLoop(bill.getAsDouble()), shooter);
    }

    public Command kill() {
        return Commands.run(() -> shooter.openLoop(0));
    }

    public Command characterize() {
        SlewRateLimiter filter = new SlewRateLimiter(0.01);
        Map<Double, Double> voltageVelocityMap = new HashMap<>();
        return Commands.runEnd(
                () -> {
                    double desiredVoltage = filter.calculate(12);
                    shooter.setVelocity(desiredVoltage);
                    voltageVelocityMap.put(desiredVoltage, shooter.getVelocity());
                },
                () -> {
                    var xArray = voltageVelocityMap.keySet().stream().toArray(Double[]::new);
                    var yArray = voltageVelocityMap.entrySet().stream().toArray(Double[]::new);
                    var xSum = 0;
                    var ySum = 0;
                    var xSquareSum = 0;
                    var xySum = 0;
                    for (int i = 0; i < xArray.length; i++) {
                        double x = xArray[i];
                        double y = yArray[i];
                        xSum += x;
                        xSquareSum += x * x;
                        ySum += y;
                        xySum += x * y;
                    }
                    // y = a + bx
                    var a =
                            (ySum * xSquareSum - xSum * xySum)
                                    / (xArray.length * xSquareSum - xSum * xSum);
                    var b =
                            (xArray.length * xySum - xSum * ySum)
                                    / (xArray.length * xSquareSum - xSum * xSum);
                    var kS = -a / b;
                    var kA = 1 / b;
                    SmartDashboard.putNumber("drivetrain kS", kS);
                    SmartDashboard.putNumber("drivetrain kA", kA);
                });
    }

    private final Set<Subsystem> requirements;

    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
        this.requirements = Set.of(shooter);
    }

    public final Shooter shooter;
}
