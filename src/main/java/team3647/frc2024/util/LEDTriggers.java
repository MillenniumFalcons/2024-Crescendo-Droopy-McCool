package team3647.frc2024.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LEDTriggers {

    public final Trigger inTrigger = new Trigger(() -> false);

    public final Trigger targetTrigger = new Trigger(() -> false);

    public final Trigger readyTrigger = new Trigger(() -> false);

    public LEDTriggers() {}
}
