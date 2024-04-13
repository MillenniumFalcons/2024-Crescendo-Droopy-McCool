package team3647.frc2024.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
                    "red", LEDConstants.SOLID_RED,
                    "blue", LEDConstants.SOLID_BLUE,
                    "intaking", LEDConstants.FLASH_ORANGE,
                    "in", LEDConstants.BREATHE_YELLOW,
                    "climb", LEDConstants.BREATHE_PINK,
                    "ready", LEDConstants.FLASH_GREEN,
                    "sotm", LEDConstants.FLASH_PURPLE);

    String defaultState = "red";

    String LEDState = defaultState;

    private CANdle m_candle;

    private LEDTriggers triggers;

    public LEDs(CANdle candle, LEDTriggers triggers) {
        this.m_candle = candle;
        m_candle.configBrightnessScalar(1);
        m_candle.configLEDType(LEDStripType.GRB);
        this.triggers = triggers;

        triggers.inOutTrigger.onTrue(setState("in"));
        triggers.inOutTrigger.onFalse(setState(defaultState));
        // triggers.outTrigger.onFalse(setState(defaultState));
        triggers.noteTrigger.onTrue(setState("intaking"));
        triggers.noteTrigger.onFalse(setState(defaultState));
        triggers.targetTrigger.whileTrue(setState("ready"));
        triggers.climbTrigger.onTrue(setState("climb"));
        triggers.climbTrigger.onFalse(setState(defaultState));
        triggers.sotmTrigger.whileTrue(setState("sotm"));
        triggers.placeholdTarget.onFalse(setState(defaultState));
    }

    private void setAnimation(Animation animation) {
        m_candle.animate(animation);
    }

    public Command setState(String state) {
        return Commands.runOnce(() -> this.LEDState = state);
    }

    @Override
    public void periodic() {
        DriverStation.getAlliance()
                .ifPresent((alliance) -> defaultState = alliance == Alliance.Blue ? "blue" : "red");
        if (LEDState == "blue" || LEDState == "red") {
            LEDState = defaultState;
        }
        setAnimation(colors.get(LEDState));
    }
}
