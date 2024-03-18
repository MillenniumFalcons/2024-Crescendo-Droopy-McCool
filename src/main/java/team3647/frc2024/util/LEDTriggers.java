package team3647.frc2024.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import team3647.frc2024.subsystems.Superstructure;
import team3647.frc2024.util.AutoDrive.DriveMode;

public class LEDTriggers {

    Superstructure superstructure;
    Supplier<DriveMode> modeSupplier;

    public LEDTriggers(Superstructure superstructure, Supplier<DriveMode> modeSupplier) {
        this.superstructure = superstructure;
        this.modeSupplier = modeSupplier;
    }

    public final Trigger noteTrigger =
            new Trigger(
                    () ->
                            modeSupplier.get() == DriveMode.INTAKE_IN_AUTO
                                    && !superstructure.hasPiece());

    public final Trigger inOutTrigger = new Trigger(() -> superstructure.hasPiece());

    public final Trigger targetTrigger = new Trigger(() -> superstructure.aimedAtSpeaker());

    public final Trigger climbTrigger = new Trigger(() -> superstructure.isClimbing());
}
