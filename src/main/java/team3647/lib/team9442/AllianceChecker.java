package team3647.lib.team9442;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AllianceChecker {
    private final List<AllianceObserver> observers = new ArrayList<>();
    private Optional<Alliance> alliance = DriverStation.getAlliance();
    private Alliance cachedColor = Alliance.Red;

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

        alliance.ifPresent(
                color -> {
                    // DriverStation.reportError("Run method? " + (cachedColor != color), false);
                    if (cachedColor != color) {
                        observers.forEach(observer -> observer.onAllianceFound(color));
                    }
                    cachedColor = color;
                });
    }
}
