package team3647.frc2024.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import team3647.frc2024.constants.LEDConstants;
import team3647.frc2024.util.LEDTriggers;
import team3647.lib.team6328.VirtualSubsystem;

public class LEDs extends VirtualSubsystem {

    /** Creates a new LEDSubsystem. */
    private Map<String, Animation> colors =
            Map.of(
                    "none", LEDConstants.BREATHE_RED,
                    "in", LEDConstants.BREATHE_YELLOW,
                    "climb", LEDConstants.BREATHE_PINK,
                    "ready", LEDConstants.FLASH_GREEN);

    String LEDState = "none";

    private CANdle m_candle;

    private LEDTriggers triggers;

    public LEDs(CANdle candle, LEDTriggers triggers) {
        this.m_candle = candle;
        this.triggers = triggers;

        triggers.inOutTrigger.onTrue(setState("in"));
        triggers.inOutTrigger.onFalse(setState("none"));
        triggers.targetTrigger.onTrue(setState("ready"));
        triggers.targetTrigger.onFalse(setState("none"));
        triggers.climbTrigger.onTrue(setState("climb"));
        triggers.climbTrigger.onFalse(setState("none"));
    }

    private void setAnimation(Animation animation) {
        m_candle.animate(animation);
    }

    public Command setState(String state) {
        return Commands.runOnce(() -> this.LEDState = state);
    }

    @Override
    public void periodic() {
        setAnimation(colors.get(LEDState));
    }
}
