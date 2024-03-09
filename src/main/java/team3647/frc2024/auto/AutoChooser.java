package team3647.frc2024.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.function.Consumer;
import team3647.frc2024.util.AllianceUpdatedObserver;

public final class AutoChooser extends SendableChooser<AutonomousMode>
        implements AllianceUpdatedObserver {

    private final Consumer<Pose2d> resetPose;
    private final AutoCommands autoCommands;

    public AutoChooser(AutoCommands autoCommands, Consumer<Pose2d> resetPose) {
        super();
        this.autoCommands = autoCommands;
        this.resetPose = resetPose;
        onChange(
                autoMode -> {
                    this.resetPose.accept(autoMode.getPathplannerPose2d());
                });
    }

    @Override
    public void onAllianceFound(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            for (AutonomousMode mode : autoCommands.getBlueModes()) {
                this.addOption(mode.getName(), mode);
            }
        } else {
            for (AutonomousMode mode : autoCommands.getRedModes()) {
                this.addOption(mode.getName(), mode);
            }
        }
    }
}
