package team3647.frc2023.robot;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2023.auto.AutoCommands;
import team3647.frc2023.auto.AutonomousMode;
import team3647.frc2023.commands.DrivetrainCommands;
import team3647.frc2023.constants.FieldConstants;
// import team3647.frc2023.auto.AutoCommands;
// // import team3647.frc2023.auto.AutoCommands;
// import team3647.frc2023.auto.AutonomousMode;
import team3647.frc2023.constants.GlobalConstants;
import team3647.frc2023.constants.IntakeConstants;
import team3647.frc2023.constants.KickerConstants;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.constants.ShooterConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.constants.TunerConstants;
import team3647.frc2023.constants.VisionConstants;
import team3647.frc2023.subsystems.Intake;
import team3647.frc2023.subsystems.Kicker;
import team3647.frc2023.subsystems.Pivot;
import team3647.frc2023.subsystems.Shooter;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.util.AprilTagPhotonVision;
import team3647.frc2023.util.AutoDrive;
import team3647.frc2023.util.AutoDrive.DriveMode;
import team3647.frc2023.util.NeuralDetector;
import team3647.frc2023.util.TargetingUtil;
import team3647.frc2023.util.VisionController;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private AutonomousMode runningMode;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        pdh.clearStickyFaults();
        scheduler.registerSubsystem(swerve, shooter, intake, kicker, pivot);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        autoCommands.registerCommands();
        runningMode = autoCommands.blueFour_S1N1N2N3;
        pivot.setEncoder(PivotConstants.kInitialAngle);
        swerve.setRobotPose(runningMode.getPathplannerPose2d());
    }

    private void configureButtonBindings() {

        mainController.buttonX.whileTrue(drivetrainCommands.characterize());
        mainController.leftBumper.whileTrue(autoDrive.setMode(DriveMode.SHOOT_ON_THE_MOVE));
        mainController
                .leftBumper
                .whileTrue(superstructure.shoot())
                .onFalse(superstructure.stowFromShoot())
                .onFalse(superstructure.ejectPiece());
        mainController
                .rightBumper
                .whileTrue(superstructure.shootManual())
                .onFalse(superstructure.stowFromShoot())
                .onFalse(superstructure.ejectPiece());
        mainController.leftBumper.onFalse(autoDrive.setMode(DriveMode.NONE));

        mainController.buttonB.and(() -> !piece.getAsBoolean()).whileTrue(superstructure.intake());
        mainController.buttonB.onFalse(superstructure.intakeCommands.kill());
        mainController.buttonB.onFalse(superstructure.kickerCommands.kill());
        mainController.buttonX.onTrue(superstructure.outtake());
        mainController.buttonX.onFalse(superstructure.intakeCommands.kill());
        mainController.buttonX.onFalse(superstructure.kickerCommands.kill());
        mainController.buttonA.onTrue(superstructure.ejectPiece());

        tofPiece.onTrue(superstructure.setPiece());
        tofPiece.and(() -> !superstructure.getIsShooting()).onTrue(superstructure.slightReverse());

        // mainController.buttonY.whileTrue(drivetrainCommands.characterize());

        // mainController.leftBumper.onTrue(superstructure.shootStow());
        // mainController.rightBumper.onTrue(superstructure.shootStow());

        // piece.whileTrue(superstructure.stowIntake());
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                drivetrainCommands.driveVisionTeleop(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        () -> false,
                        () -> true,
                        autoDrive::getMode,
                        autoDrive::getEnabled,
                        autoDrive::getVelocities));
        pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());
        intake.setDefaultCommand(superstructure.intakeCommands.kill());
        kicker.setDefaultCommand(superstructure.kickerCommands.kill());
        shooter.setDefaultCommand(superstructure.shooterCommands.kill());
    }

    public void teleopInit() {}

    void configTestCommands() {}

    public void configureSmartDashboardLogging() {
        SmartDashboard.putNumber("pivot interp angle", 40);
        printer.addBoolean("tof", () -> tofPiece.getAsBoolean());
        printer.addBoolean("shooht", () -> superstructure.getIsShooting());
        // printer.addDouble("auto drive", () -> autoDrive.getVelocities().dtheta);
    }

    public Command getAutonomousCommand() {
        return runningMode.getAutoCommand();
    }

    private final Joysticks mainController = new Joysticks(0);

    private TimeOfFlight tof = new TimeOfFlight(GlobalConstants.SensorIds.tofID);

    public final SwerveDrive swerve =
            new SwerveDrive(
                    TunerConstants.DrivetrainConstants,
                    SwerveDriveConstants.kDriveKinematics,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
                    GlobalConstants.kDt,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight);

    public final Shooter shooter =
            new Shooter(
                    ShooterConstants.kTopRoller,
                    ShooterConstants.kBottomRoller,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    1,
                    ShooterConstants.kNominalVoltage,
                    0.02,
                    ShooterConstants.ff);

    public final Kicker kicker =
            new Kicker(KickerConstants.kMaster, 1, 1, KickerConstants.kNominalVoltage, 0.02);

    public final Intake intake =
            new Intake(
                    IntakeConstants.kMaster,
                    IntakeConstants.kSlave,
                    1,
                    1,
                    IntakeConstants.kNominalVoltage,
                    0.02);

    public final Pivot pivot =
            new Pivot(
                    PivotConstants.kMaster,
                    PivotConstants.kNativeVelToDPS,
                    PivotConstants.kNativePosToDegrees,
                    PivotConstants.kMinDegree,
                    PivotConstants.kMaxDegree,
                    PivotConstants.nominalVoltage,
                    PivotConstants.maxKG,
                    0.02);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final DrivetrainCommands drivetrainCommands = new DrivetrainCommands(swerve);

    public final AprilTagPhotonVision ar_doo_cam =
            new AprilTagPhotonVision(
                    VisionConstants.photonName,
                    VisionConstants.robotToPhotonCam,
                    swerve::getOdoPose);

    private VisionController visionController =
            new VisionController(swerve::addVisionData, swerve, ar_doo_cam);

    public final NeuralDetector detector = new NeuralDetector(VisionConstants.limelightName);

    public final TargetingUtil targetingUtil =
            new TargetingUtil(
                    FieldConstants.kBlueSpeaker,
                    swerve::getOdoPose,
                    swerve::getFieldRelativeChassisSpeeds,
                    PivotConstants.robotToPivot);

    public final AutoDrive autoDrive = new AutoDrive(swerve, detector, targetingUtil);

    public final Superstructure superstructure =
            new Superstructure(
                    intake,
                    kicker,
                    shooter,
                    pivot,
                    autoDrive::getPivotAngle,
                    targetingUtil.exitVelocity());

    public final AutoCommands autoCommands =
            new AutoCommands(swerve, autoDrive::getVelocities, superstructure, targetingUtil);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    final GroupPrinter printer = GroupPrinter.getInstance();

    Trigger piece = new Trigger(() -> superstructure.getPiece());

    Trigger tofPiece =
            new Trigger(
                    () ->
                            (tof.getRange() < 100
                                    && !superstructure.getIsShooting()
                                    && !DriverStation.isAutonomous()));
}
