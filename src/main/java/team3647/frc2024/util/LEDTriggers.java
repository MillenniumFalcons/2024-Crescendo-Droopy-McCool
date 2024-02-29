package team3647.frc2024.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2024.subsystems.Superstructure;

public class LEDTriggers {

    Superstructure superstructure;

    public LEDTriggers(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    public final Trigger inOutTrigger = new Trigger(() -> superstructure.hasPiece());

    public final Trigger targetTrigger = new Trigger(() -> superstructure.aimedAtSpeaker());

    public final Trigger climbTrigger = new Trigger(() -> superstructure.isClimbing());
}
