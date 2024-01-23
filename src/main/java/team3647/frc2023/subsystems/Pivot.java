package team3647.frc2023.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {

    private double minAngle;
    private double maxAngle;

    private double maxKG;

    private Pose3d pose = new Pose3d(new Translation3d(), new Rotation3d(0, 0, 0));

    public Pivot(
            TalonFX master,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double minAngle,
            double maxAngle,
            double nominalVoltage,
            double maxKG,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.maxKG = maxKG;
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    @Override
    public void periodic() {
        super.periodic();
        pose =
                new Pose3d(
                        new Translation3d(),
                        new Rotation3d(0, Units.degreesToRadians(getAngle()), 0));
        Logger.recordOutput(getName() + "/pose", pose);
        Logger.recordOutput(getName() + "/angle", getAngle());
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
        Logger.recordOutput(getName() + "/demand", demand);
    }

    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    public double getAngle() {
        return super.getPosition();
    }

    public void setAngle(double angle) {
        double desiredAngle = MathUtil.clamp(angle, minAngle, maxAngle);
        var ffvolts = maxKG * Math.cos(desiredAngle);
        super.setPositionMotionMagic(desiredAngle, ffvolts);
        Logger.recordOutput(getName() + "/angle", angle);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
