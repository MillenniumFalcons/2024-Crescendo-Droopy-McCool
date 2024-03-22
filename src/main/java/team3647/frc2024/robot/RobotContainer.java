package team3647.frc2024.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import team3647.frc2024.auto.AutonomousMode;
import team3647.frc2024.commands.ClimbCommands;
import team3647.frc2024.commands.DrivetrainCommands;
import team3647.frc2024.constants.ChurroConstants;
import team3647.frc2024.constants.ClimbConstants;
import team3647.frc2024.constants.FieldConstants;
import team3647.frc2024.constants.GlobalConstants;
import team3647.frc2024.constants.IntakeConstants;
import team3647.frc2024.constants.KickerConstants;
import team3647.frc2024.constants.PivotConstants;
import team3647.frc2024.constants.ShooterConstants;
import team3647.frc2024.constants.SwerveDriveConstants;
import team3647.frc2024.constants.TunerConstants;
import team3647.frc2024.constants.VisionConstants;
import team3647.frc2024.constants.WristConstants;
import team3647.frc2024.subsystems.Churro;
import team3647.frc2024.subsystems.Climb;
import team3647.frc2024.subsystems.Intake;
import team3647.frc2024.subsystems.Kicker;
import team3647.frc2024.subsystems.Pivot;
import team3647.frc2024.subsystems.ShooterLeft;
import team3647.frc2024.subsystems.ShooterRight;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.frc2024.subsystems.Wrist;
import team3647.frc2024.util.AprilTagPhotonVision;
import team3647.frc2024.util.AutoDrive;
import team3647.frc2024.util.NeuralDetector;
import team3647.frc2024.util.NeuralDetectorPhotonVision;
import team3647.frc2024.util.RobotTracker;
import team3647.frc2024.util.TargetingUtil;
import team3647.frc2024.util.VisionController;
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

        // to change side change both auto and stuff in robot tracker

        pdh.clearStickyFaults();
        scheduler.registerSubsystem(
                swerve,
                shooterRight,
                shooterLeft,
                intake,
                wrist,
                kicker,
                pivot,
                printer,
                climb,
                churro);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        runningMode = new AutonomousMode(new InstantCommand(), new Pose2d());
        pivot.setEncoder(PivotConstants.kInitialAngle);
        wrist.setEncoder(WristConstants.kInitialDegree);
        climb.setEncoder(0);
        churro.setEncoder(ChurroConstants.kInitialDegree);
        swerve.setRobotPose(runningMode.getPathplannerPose2d());

        RobotController.setBrownoutVoltage(5.5);
    }

    private void configureButtonBindings() {

        // shooter

        mainController.dPadUp.whileTrue(
                shooterLeft.runQuasiTest(
                        edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        mainController.dPadDown.whileTrue(
                shooterLeft.runQuasiTest(
                        edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        mainController.dPadLeft.whileTrue(
                shooterLeft.runDynamTest(
                        edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        mainController.dPadRight.whileTrue(
                shooterLeft.runDynamTest(
                        edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
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
    }

    public void teleopInit() {}

    void configTestCommands() {}

    public void configureSmartDashboardLogging() {
        SmartDashboard.putNumber("pivot output", 0);
        // SmartDashboard.putNumber("wrist kg", 0);
        // printer.addDouble("wanted pivot", superstructure::getWantedPivot);
        printer.addDouble("back tof", pivot::tofBack);
        printer.addBoolean("front tof", pivot::frontPiece);
        // printer.addDouble("wrist", wrist::getAngle);
        printer.addDouble("pivot", pivot::getAngle);
        printer.addDouble("churo", churro::getAngle);
        SmartDashboard.putNumber("bill", 3.93);
        // printer.addBoolean("under stage", swerve::underStage);
        // printer.addBoolean("set piece", () -> setPiece.getAsBoolean());
        // printer.addBoolean("swerve aimed", autoDrive::swerveAimed);
        // printer.addBoolean("spun up", superstructure::flywheelReadY);
        // printer.addBoolean("pviot ready", superstructure::pivotReady);
        // printer.addDouble("flywheel speed", shooterLeft::getVelocity);
        SmartDashboard.putNumber("pivot interp angle", 40);
        SmartDashboard.putNumber("shoot speed", 15);
        SmartDashboard.putNumber("differential", 1.1);
        printer.addDouble("shooter distance", targetingUtil::distance);
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
        return runningMode.getAutoCommand();
    }

    private final Joysticks mainController = new Joysticks(0);
    private final Joysticks coController = new Joysticks(1);

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

    public final ShooterRight shooterRight =
            new ShooterRight(
                    ShooterConstants.kRightRoller,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    1,
                    ShooterConstants.kNominalVoltage,
                    0.02,
                    ShooterConstants.ff);

    public final ShooterLeft shooterLeft =
            new ShooterLeft(
                    ShooterConstants.kLeftRoller,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    1,
                    ShooterConstants.kNominalVoltage,
                    0.02,
                    ShooterConstants.ff);

    public final Wrist wrist =
            new Wrist(
                    WristConstants.kMaster,
                    WristConstants.kNativeVelToDPS,
                    WristConstants.kNativePosToDegrees,
                    WristConstants.kMinDegree,
                    WristConstants.kMaxDegree,
                    WristConstants.nominalVoltage,
                    WristConstants.kG,
                    0.02);

    public final Kicker kicker =
            new Kicker(KickerConstants.kMaster, 1, 1, KickerConstants.kNominalVoltage, 0.02);

    public final Intake intake =
            new Intake(IntakeConstants.kMaster, 1, 1, IntakeConstants.kNominalVoltage, 0.02);

    public final Pivot pivot =
            new Pivot(
                    PivotConstants.kMaster,
                    PivotConstants.kSlave,
                    PivotConstants.kNativeVelToDPS,
                    PivotConstants.kNativePosToDegrees,
                    PivotConstants.kMinDegree,
                    PivotConstants.kMaxDegree,
                    PivotConstants.kMaxDegreeUnderStage,
                    swerve::underStage,
                    PivotConstants.nominalVoltage,
                    PivotConstants.maxKG,
                    0.02,
                    PivotConstants.tofBack,
                    PivotConstants.tofFront);

    private final Climb climb =
            new Climb(
                    ClimbConstants.kLeft,
                    ClimbConstants.kRight,
                    1,
                    1,
                    ClimbConstants.kMinLength,
                    ClimbConstants.kMaxDegreeLength,
                    ClimbConstants.nominalVoltage,
                    0,
                    0.02);

    public final Churro churro =
            new Churro(
                    ChurroConstants.kMaster,
                    ChurroConstants.kNativeVelToDPS,
                    ChurroConstants.kNativePosToDegrees,
                    ChurroConstants.kMinDegree,
                    ChurroConstants.kMaxDegree,
                    ChurroConstants.nominalVoltage,
                    ChurroConstants.kG,
                    0.02);

    private final ClimbCommands climbCommands = new ClimbCommands(climb);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final DrivetrainCommands drivetrainCommands = new DrivetrainCommands(swerve);

    public final AprilTagPhotonVision backLeft =
            new AprilTagPhotonVision(VisionConstants.backLeft, VisionConstants.robotToBackLeft);

    public final AprilTagPhotonVision backRight =
            new AprilTagPhotonVision(VisionConstants.backRight, VisionConstants.robotToBackRight);

    public final AprilTagPhotonVision left =
            new AprilTagPhotonVision(VisionConstants.left, VisionConstants.robotToLeft);

    public final AprilTagPhotonVision right =
            new AprilTagPhotonVision(VisionConstants.right, VisionConstants.robotToRight);

    public final AprilTagPhotonVision zoom =
            new AprilTagPhotonVision(VisionConstants.zoom, VisionConstants.robotToZoom)
                    .withPriority(true);

    private final VisionController visionController =
            new VisionController(
                    swerve::addVisionData,
                    swerve::shouldAddData,
                    swerve::seedFieldRelative,
                    coController.buttonA,
                    coController.buttonX,
                    backLeft,
                    backRight,
                    left,
                    right,
                    zoom);

    public final NeuralDetector detector = new NeuralDetectorPhotonVision(VisionConstants.driver);

    public final RobotTracker tracker =
            new RobotTracker(
                    FieldConstants.kBlueSpeaker,
                    FieldConstants.kBlueAmp,
                    PivotConstants.robotToPivot2d,
                    swerve::getOdoPose,
                    swerve::getChassisSpeeds,
                    false);

    public final TargetingUtil targetingUtil = new TargetingUtil(tracker);

    public final AutoDrive autoDrive = new AutoDrive(swerve, detector, targetingUtil);
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    final GroupPrinter printer = GroupPrinter.getInstance();

    private final Trigger climbing = new Trigger(() -> climb.getPosition() > 1);

    private final BooleanSupplier isIntaking =
            () -> mainController.leftBumper.getAsBoolean() && !DriverStation.isAutonomous();
}
