package team3647.frc2024.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import team3647.frc2024.auto.AutoCommands;
import team3647.frc2024.auto.AutonomousMode;
import team3647.frc2024.commands.ClimbCommands;
import team3647.frc2024.commands.DrivetrainCommands;
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
import team3647.frc2024.subsystems.Climb;
import team3647.frc2024.subsystems.Intake;
import team3647.frc2024.subsystems.Kicker;
import team3647.frc2024.subsystems.Pivot;
import team3647.frc2024.subsystems.ShooterLeft;
import team3647.frc2024.subsystems.ShooterRight;
import team3647.frc2024.subsystems.Superstructure;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.frc2024.subsystems.Wrist;
import team3647.frc2024.util.AprilTagPhotonVision;
import team3647.frc2024.util.AutoDrive;
import team3647.frc2024.util.AutoDrive.DriveMode;
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
                swerve, shooterRight, shooterLeft, intake, wrist, kicker, pivot, printer, climb);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        autoCommands.registerCommands();
        runningMode = autoCommands.blueFour_S1F1F2F3;
        pivot.setEncoder(PivotConstants.kInitialAngle);
        wrist.setEncoder(WristConstants.kInitialDegree);
        climb.setEncoder(0);
        swerve.setRobotPose(runningMode.getPathplannerPose2d());
    }

    private void configureButtonBindings() {

        mainController.buttonX.whileTrue(superstructure.kickerCommands.unkick());
        mainController.buttonX.onFalse(superstructure.kickerCommands.kill());
        mainController.rightTrigger.whileTrue(autoDrive.setMode(DriveMode.SHOOT_ON_THE_MOVE));
        // mainController.leftTrigger.whileTrue(autoDrive.setMode(DriveMode.SHOOT_AT_AMP));
        mainController
                .rightTrigger
                .and(() -> superstructure.getPiece() || mainController.buttonY.getAsBoolean())
                .whileTrue(superstructure.shoot())
                .onFalse(superstructure.stowFromShoot())
                .onFalse(superstructure.ejectPiece());
        mainController
                .rightBumper
                .and(() -> superstructure.getPiece() || mainController.buttonY.getAsBoolean())
                .whileTrue(superstructure.batterShot())
                .onFalse(superstructure.stowFromShoot())
                .onFalse(superstructure.ejectPiece());
        mainController
                .leftTrigger
                .and(() -> superstructure.getPiece())
                .whileTrue(superstructure.shootAmp())
                .onFalse(superstructure.stowFromShoot())
                .onFalse(superstructure.ejectPiece());
        mainController.rightTrigger.onFalse(autoDrive.setMode(DriveMode.NONE));
        mainController.leftTrigger.onFalse(autoDrive.setMode(DriveMode.NONE));

        mainController.leftMidButton.onTrue(autoDrive.enable());
        mainController.rightMidButton.onTrue(autoDrive.disable());

        mainController
                .leftBumper
                .and(() -> detector.hasTarget())
                .whileTrue(autoDrive.setMode(DriveMode.INTAKE_FLOOR_PIECE));
        mainController
                .leftBumper
                .and(() -> !superstructure.getPiece())
                .whileTrue(superstructure.intake().until(setPiece.and(isIntaking)))
                .whileTrue(superstructure.pivotCommands.setAngle(() -> 20));
        setPiece.and(isIntaking)
                .onTrue(superstructure.setPiece())
                .onTrue(superstructure.passToShooter());
        mainController
                .leftBumper
                .or(() -> superstructure.getPiece())
                .onFalse(superstructure.stowIntake())
                .onFalse(superstructure.kickerCommands.kill());
        mainController
                .leftBumper
                .and(() -> detector.hasTarget())
                .onFalse(autoDrive.setMode(DriveMode.NONE));

        mainController.buttonA.onTrue(superstructure.ejectPiece());

        mainController.dPadLeft.onTrue(targetingUtil.offsetUp());
        mainController.dPadRight.onTrue(targetingUtil.offsetDown());

        mainController.dPadUp.whileTrue(climbCommands.goUp());
        mainController.dPadUp.onFalse(climbCommands.kill());
        mainController.dPadDown.whileTrue(climbCommands.goDown());
        mainController.dPadDown.onFalse(climbCommands.kill());

        // characterization

        // swerve

        // mainController.dPadUp.whileTrue(swerve.runDriveQuasiTest(Direction.kForward));
        // mainController.dPadDown.whileTrue(swerve.runDriveQuasiTest(Direction.kReverse));

        // mainController.dPadLeft.whileTrue(swerve.runDriveDynamTest(Direction.kForward));
        // mainController.dPadRight.whileTrue(swerve.runDriveDynamTest(Direction.kReverse));

        // mainController.buttonY.whileTrue(swerve.runSteerQuasiTest(Direction.kForward));
        // mainController.buttonA.whileTrue(swerve.runSteerQuasiTest(Direction.kReverse));

        // mainController.buttonX.whileTrue(swerve.runSteerDynamTest(Direction.kForward));
        // mainController.buttonB.whileTrue(swerve.runSteerDynamTest(Direction.kReverse));

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
        pivot.setDefaultCommand(superstructure.prep());
        intake.setDefaultCommand(superstructure.intakeCommands.kill());
        kicker.setDefaultCommand(superstructure.kickerCommands.kill());
        wrist.setDefaultCommand(superstructure.wristCommands.holdPositionAtCall());
        shooterRight.setDefaultCommand(superstructure.shooterCommands.killRight());
        shooterLeft.setDefaultCommand(superstructure.shooterCommands.killLeft());
    }

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
        printer.addBoolean("swerve aimed", autoDrive::swerveAimed);
        printer.addBoolean("spun up", superstructure::flywheelReadY);
        printer.addBoolean("pviot ready", superstructure::pivotReady);
        SmartDashboard.putNumber("pivot interp angle", 40);
        printer.addDouble("shooter distance squared", targetingUtil::distance);
        // printer.addBoolean("current sensing", () -> autoCommands.currentYes.getAsBoolean());
        // printer.addBoolean("wrist at stow", superstructure::wristAtStow);
        // printer.addDouble("climb len", () -> climb.getLength());
        // printer.addBoolean("shooter ready", superstructure::flywheelReadY);
        // printer.addBoolean("pivot ready", superstructure::pivotReady);
        // printer.addBoolean("swerve ready", superstructure::swerveReady);
        // printer.addDouble("tx", detector::getTX);
        SmartDashboard.putNumber("shoot speed", 30);
        SmartDashboard.putNumber("ratio", 1);
        // printer.addDouble("auto drive", () -> autoDrive.getVelocities().dtheta);
    }

    public Command getAutonomousCommand() {
        return runningMode.getAutoCommand();
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

    private final VisionController visionController =
            new VisionController(
                    swerve::addVisionData, swerve::shouldAddData, backLeft, left, backRight, right);

    //     private final LEDs LEDs = new LEDs(LEDConstants.m_candle);

    public final NeuralDetector detector = new NeuralDetectorPhotonVision(VisionConstants.driver);

    public final TargetingUtil targetingUtil =
            new TargetingUtil(
                    new RobotTracker(
                            FieldConstants.kBlueSpeaker,
                            FieldConstants.kBlueAmp,
                            swerve::getOdoPose,
                            swerve::getChassisSpeeds,
                            PivotConstants.robotToPivot2d,
                            false));

    public final AutoDrive autoDrive = new AutoDrive(swerve, detector, targetingUtil);

    public final Superstructure superstructure =
            new Superstructure(
                    intake,
                    kicker,
                    shooterRight,
                    shooterLeft,
                    pivot,
                    wrist,
                    autoDrive::getPivotAngle,
                    autoDrive::getShootSpeed,
                    targetingUtil.exitVelocity(),
                    autoDrive::swerveAimed);

    public final AutoCommands autoCommands =
            new AutoCommands(swerve, autoDrive::getVelocities, superstructure, targetingUtil);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    final GroupPrinter printer = GroupPrinter.getInstance();

    private final Trigger piece = new Trigger(() -> superstructure.getPiece());

    private final Trigger setPiece =
            new Trigger(() -> intake.getMasterCurrent() > 45 && wrist.getAngle() < 5)
                    .debounce(0.06)
                    .or(mainController.buttonB);

    private final BooleanSupplier isIntaking =
            () -> mainController.leftBumper.getAsBoolean() && !DriverStation.isAutonomous();
}
