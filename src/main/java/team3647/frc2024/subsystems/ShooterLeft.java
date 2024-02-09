package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team3647.frc2024.util.ModifiedSignalLogger;
import team3647.lib.TalonFXSubsystem;

public class ShooterLeft extends TalonFXSubsystem {
    private final SimpleMotorFeedforward ff;

    private SysIdRoutine m_shooterSysIdRoutine;

    public ShooterLeft(
            TalonFX master,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.ff = ff;

        this.m_shooterSysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> volts) -> setVoltage(volts.in(Units.Volts)),
                                null,
                                this));
    }

    public Command runQuasiTest(Direction direction) {
        return m_shooterSysIdRoutine.quasistatic(direction);
    }

    public Command runDynamTest(SysIdRoutine.Direction direction) {
        return m_shooterSysIdRoutine.dynamic(direction);
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
    }

    public void setVelocity(double velocity) {
        super.setVelocity(velocity, ff.calculate(velocity));
    }

    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    public boolean velocityReached(double setpoint, double threshold) {
        return Math.abs(super.getVelocity() - setpoint) < threshold;
    }

    @Override
    public String getName() {
        return "Shooter Left";
    }
}
