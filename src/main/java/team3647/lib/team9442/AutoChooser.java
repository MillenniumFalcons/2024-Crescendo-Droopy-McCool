package team3647.lib.team9442;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.inputs.LoggedDriverStation.DriverStationInputs;

import team3647.frc2024.auto.AutoCommands;
import team3647.frc2024.auto.AutonomousMode;

public class AutoChooser extends SendableChooser<AutonomousMode> implements AllianceObserver {

    public AutoCommands autoCommands;
    Consumer<Pose2d> setStartPose;
    List<AutonomousMode> autosList;
    Alliance chachedColor = Alliance.Red;

    public AutoChooser(AutoCommands commands, Consumer<Pose2d> setStartPose) {
        super();
        this.autoCommands = commands;
        this.setStartPose = setStartPose;
        autosList = new ArrayList<AutonomousMode>();
        onChange(
                (mode) -> {
                    setStartPose.accept(getSelected().getPathplannerPose2d());
                    SmartDashboard.putString("sellected", getSelected().getName());
                });
        autosList = autoCommands.redAutoModes;
        setDefaultOption("rDEFAULT AUTO CHANE THIS red full center", autosList.get(0));
    }

    @Override
    public void onAllianceFound(Alliance color) {
        if(color != chachedColor){
            setStartPose.accept(getSelected().getPathplannerPose2d());
        }
        
        
        autosList = color == Alliance.Blue ? autoCommands.blueAutoModes : autoCommands.redAutoModes;
        addAutos();
        chachedColor = color;
    }

    public void addAutos() {
        for (AutonomousMode mode : this.autosList) {
            addOption(mode.getName(), mode);
        }
    }
}
