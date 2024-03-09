package team3647.frc2024.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface AllianceUpdatedObserver {
    public void onAllianceFound(Alliance alliance);
}
