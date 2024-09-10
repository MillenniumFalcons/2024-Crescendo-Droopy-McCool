package team3647.lib.team9442;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.frc2024.auto.AutoCommands;
import team3647.frc2024.auto.AutonomousMode;

public class AutoChooser extends SendableChooser<AutonomousMode> implements AllianceObserver {

    public AutoCommands autoCommands;
    Consumer<Pose2d> setStartPose;
    List<AutonomousMode> autosList;
    

    public AutoChooser(AutoCommands commands, Consumer<Pose2d> setStartPose){
        super();
        this.autoCommands = commands;
        this.setStartPose = setStartPose;
        autosList = new ArrayList<AutonomousMode>();
        onChange((mode) -> {setStartPose.accept(getSelected().getPathplannerPose2d());
                            SmartDashboard.putString("sellected", getSelected().getName());});
        autosList = autoCommands.redAutoModes;
        
    }

    @Override
    public void onAllianceFound(Alliance color) {
        System.out.println("FUCKINGHELL MTHE MTIODINF D");
        setStartPose.accept(getSelected().getPathplannerPose2d());
        autosList = color == Alliance.Blue? 
                autoCommands.blueAutoModes: 
                autoCommands.redAutoModes;
        addAutos();
    }

    public void addAutos(){
        DriverStation.reportError("THE METHOD RUNS FOR FUCKS AKE " + autosList.size(), false);
        for(AutonomousMode mode : this.autosList){
            DriverStation.reportError("loopasoidfjoeaijsdkfjaoisdljfalei", false);
            DriverStation.reportError(mode.getName() + "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", false);
            addOption(mode.getName(), mode);
        }
    }
}
