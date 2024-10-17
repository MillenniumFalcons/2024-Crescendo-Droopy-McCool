package team3647.lib;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Simshit {
     private static Map<AllianceStationID, Optional<Alliance>> m_allianceMap = Map.of(
      AllianceStationID.Unknown, Optional.empty(),
      AllianceStationID.Red1, Optional.of(Alliance.Red),
      AllianceStationID.Red2, Optional.of(Alliance.Red),
      AllianceStationID.Red3, Optional.of(Alliance.Red),
      AllianceStationID.Blue1, Optional.of(Alliance.Blue),
      AllianceStationID.Blue2, Optional.of(Alliance.Blue),
      AllianceStationID.Blue3, Optional.of(Alliance.Blue));
    
    public static Optional<Alliance> toAlliance(AllianceStationID id){
        return m_allianceMap.get(id);
    }
}
