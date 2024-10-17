package team3647.lib.team9442;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import team3647.lib.Simshit;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class AllianceChecker {
    private final List<AllianceObserver> observers = new ArrayList<>();
    private Optional<Alliance> alliance = DriverStation.getAlliance();
    private Optional<Alliance> simAlliance = Simshit.toAlliance(DriverStationSim.getAllianceStationId());


    public void registerObserver(AllianceObserver observer) {
        observers.add(observer);
    }

    public void registerObservers(AllianceObserver... addObservers) {
        for (final AllianceObserver o : addObservers) {
            observers.add(o);
        }
    }

    public void periodic() {
        alliance = DriverStation.getAlliance();
        simAlliance = Simshit.toAlliance(DriverStationSim.getAllianceStationId());

        
       if (RobotBase.isReal()) {
         alliance.ifPresent(color -> observers.forEach(observer -> observer.onAllianceFound(color)));
       }
        if(RobotBase.isSimulation()){
            
            simAlliance.ifPresent(color -> observers.forEach(observer -> observer.onAllianceFound(color)));
        }
    }
}
