package team3647.frc2024.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team3647.frc2024.constants.GlobalConstants;
import team3647.frc2024.constants.SwerveDriveConstants;
import team3647.frc2024.constants.TunerConstants;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // to change side change both auto and stuff in robot tracker

        pdh.clearStickyFaults();
        scheduler.registerSubsystem(swerve);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
    }

    private void configureButtonBindings() {

        // characterization

        // swerve

        mainController.dPadUp.whileTrue(swerve.runDriveQuasiTest(Direction.kForward));
        mainController.dPadDown.whileTrue(swerve.runDriveQuasiTest(Direction.kReverse));

        mainController.dPadLeft.whileTrue(swerve.runDriveDynamTest(Direction.kForward));
        mainController.dPadRight.whileTrue(swerve.runDriveDynamTest(Direction.kReverse));

        mainController.buttonY.whileTrue(swerve.runSteerQuasiTest(Direction.kForward));
        mainController.buttonA.whileTrue(swerve.runSteerQuasiTest(Direction.kReverse));

        mainController.buttonX.whileTrue(swerve.runSteerDynamTest(Direction.kForward));
        mainController.buttonB.whileTrue(swerve.runSteerDynamTest(Direction.kReverse));

        // shooter

        // mainController.dPadUp.whileTrue(
        //         shooterLeft.runQuasiTest(
        //                 edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        // mainController.dPadDown.whileTrue(
        //         shooterLeft.runQuasiTest(
        //                 edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        // mainController.dPadLeft.whileTrue(
        //         shooterLeft.runDynamTest(
        //                 edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        // mainController.dPadRight.whileTrue(
        //         shooterLeft.runDynamTest(
        //                 edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
    }

    private void configureDefaultCommands() {}

    public void teleopInit() {}

    void configTestCommands() {}

    public void configureSmartDashboardLogging() {
        // SmartDashboard.putNumber("wrist kg", 0);
        // printer.addDouble("wanted pivot", superstructure::getWantedPivot);
        // printer.addDouble("back tof", pivot::tofBack);
        // printer.addBoolean("front tof", pivot::frontPiece);
        // printer.addDouble("wrist", wrist::getAngle);
        // printer.addDouble("pivot", pivot::getAngle);
        // printer.addBoolean("under stage", swerve::underStage);
        // printer.addBoolean("set piece", () -> setPiece.getAsBoolean());
        // printer.addBoolean("swerve aimed", autoDrive::swerveAimed);
        // printer.addBoolean("spun up", superstructure::flywheelReadY);
        // printer.addBoolean("pviot ready", superstructure::pivotReady);
        // printer.addDouble("flywheel speed", shooterLeft::getVelocity);
        // SmartDashboard.putNumber("pivot interp angle", 40);
        // printer.addDouble("shooter distance squared", targetingUtil::distance);
        // printer.addBoolean("current sensing", () -> autoCommands.currentYes.getAsBoolean());
        // printer.addBoolean("wrist at stow", superstructure::wristAtStow);
        // printer.addDouble("climb len", () -> climb.getLength());
        // printer.addBoolean("shooter ready", superstructure::flywheelReadY);
        // printer.addBoolean("pivot ready", superstructure::pivotReady);
        // printer.addBoolean("swerve ready", superstructure::swerveReady);
        // printer.addDouble("tx", detector::getTX);
        // SmartDashboard.putNumber("shoot speed", 30);
        // SmartDashboard.putNumber("ratio", 1);
        // printer.addDouble("auto drive", () -> autoDrive.getVelocities().dtheta);
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

    private final Joysticks mainController = new Joysticks(0);

    public final SwerveDrive swerve =
            new SwerveDrive(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.kSpeedAt12VoltsMps,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
                    GlobalConstants.kDt,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    final GroupPrinter printer = GroupPrinter.getInstance();
}
