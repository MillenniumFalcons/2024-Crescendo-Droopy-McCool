package team3647.frc2024.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.sql.Driver;
import java.util.function.BooleanSupplier;
import team3647.frc2024.auto.AutoCommands;
import team3647.frc2024.auto.AutonomousMode;
import team3647.frc2024.commands.ClimbCommands;
import team3647.frc2024.commands.DrivetrainCommands;
import team3647.frc2024.constants.ChurroConstants;
import team3647.frc2024.constants.ClimbConstants;
import team3647.frc2024.constants.FieldConstants;
import team3647.frc2024.constants.GlobalConstants;
import team3647.frc2024.constants.IntakeConstants;
import team3647.frc2024.constants.KickerConstants;
import team3647.frc2024.constants.LEDConstants;
import team3647.frc2024.constants.PivotConstants;
import team3647.frc2024.constants.ShooterConstants;
import team3647.frc2024.constants.SwerveDriveConstants;
import team3647.frc2024.constants.TunerConstants;
import team3647.frc2024.constants.VisionConstants;
import team3647.frc2024.constants.WristConstants;
import team3647.frc2024.subsystems.Churro;
import team3647.frc2024.subsystems.ClimbLeft;
import team3647.frc2024.subsystems.ClimbRight;
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
import team3647.frc2024.util.LEDTriggers;
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
                climbLeft,
                climbRight,
                churro);
        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        autoCommands.registerCommands();
        runningMode = autoCommands.redSix_S1F1F2N1N2N3;
        pivot.setEncoder(PivotConstants.kInitialAngle);
        wrist.setEncoder(WristConstants.kInitialDegree);
        climbLeft.setEncoder(ClimbConstants.kInitialLength);
        climbRight.setEncoder(ClimbConstants.kInitialLength);
        churro.setEncoder(ChurroConstants.kInitialDegree);
        swerve.setRobotPose(runningMode.getPathplannerPose2d());

        RobotController.setBrownoutVoltage(5.5);
    }

    private void configureButtonBindings() {

        ledTriggers.inOutTrigger.onTrue(mainController.rumble());

        coController.dPadUp.onTrue(superstructure.currentUp());
        coController.dPadDown.onTrue(superstructure.currentDown());

        coController.leftBumper.whileTrue(superstructure.sourceIntake());
        coController.leftBumper.onFalse(superstructure.stowFromSourceIntake());

        mainController
                .buttonX
                .whileTrue(superstructure.kickerCommands.unkick())
                .whileTrue(superstructure.intakeCommands.spitOut());
        mainController
                .buttonX
                .onFalse(superstructure.kickerCommands.kill())
                .onFalse(superstructure.intakeCommands.kill());

        // coController.buttonY.onTrue(superstructure.setShootModeClean());
        coController.buttonX.onTrue(superstructure.setShootModeMoving());
        coController.buttonA.onTrue(superstructure.setShootModeFeed());
        coController.buttonB.onTrue(superstructure.setShootModeStationary());

        mainController.rightTrigger.whileTrue(
                autoDrive.setMode(() -> superstructure.getWantedShootingMode()));
        mainController
                .rightTrigger
                .and(() -> !swerve.getIsAccel())
                .onTrue(swerve.setAccelLimit(SwerveDriveConstants.shootingAccel))
                .onFalse(swerve.setAccelLimit(SwerveDriveConstants.defaultAccel));
        mainController
                .leftTrigger
                .and(goodToAmp)
                .whileTrue(autoDrive.setMode(DriveMode.SHOOT_AT_AMP));
        // mainController.leftTrigger.onFalse(autoCommands.pathToAmp(tracker.getColor()));
        mainController
                .rightTrigger
                .and(
                        () ->
                                (!superstructure.getIsIntaking()
                                        && autoDrive.getMode() != DriveMode.CLEAN))
                .whileTrue(superstructure.shoot())
                .onFalse(
                        superstructure
                                .stowFromShoot()
                                .andThen(superstructure.ejectPiece())
                                .unless(mainController.buttonY));

        mainController
                .rightTrigger
                .and(() -> (autoDrive.getMode() == DriveMode.CLEAN))
                .whileTrue(superstructure.cleanShoot())
                .onFalse(
                        superstructure
                                .stowFromShoot()
                                .andThen(superstructure.ejectPiece())
                                .unless(mainController.buttonY));
        // coController
        //         .buttonX
        //         .whileTrue(autoCommands.pathToTrapTest())
        //         .onFalse(superstructure.ejectPiece());
        // .onFalse(superstructure.ejectPiece());
        mainController
                .rightBumper
                .and(() -> (!superstructure.getIsIntaking()))
                .whileTrue(superstructure.batterShot())
                .onFalse(
                        superstructure
                                .stowFromBatterShoot()
                                .andThen(superstructure.ejectPiece())
                                .unless(mainController.buttonY));
        mainController
                .leftTrigger
                .and(goodToAmp)
                .and(() -> (!superstructure.getIsIntaking()))
                .whileTrue(superstructure.shootAmp(swerve::getPoseY))
                .onFalse(
                        superstructure
                                .stowFromAmpShoot()
                                .andThen(superstructure.ejectPiece())
                                .unless(mainController.buttonY));
        mainController.rightTrigger.onFalse(
                Commands.sequence(Commands.waitSeconds(0.6), autoDrive.setMode(DriveMode.NONE)));
        mainController
                .leftTrigger
                .and(goodToAmp)
                .onFalse(
                        Commands.sequence(
                                Commands.waitSeconds(1), autoDrive.setMode(DriveMode.NONE)));

        coController.leftMidButton.onTrue(autoDrive.enable()); // the one with the rectangles
        coController.rightMidButton.onTrue(autoDrive.disable()); // the one with the lines

        mainController
                .leftBumper
                .and(() -> detector.hasTarget() && autoDrive.getMode() != DriveMode.CLEAN)
                .whileTrue(autoDrive.setMode(DriveMode.INTAKE_IN_AUTO));
        // mainController
        //         .leftBumper
        //         .and(
        //                 () ->
        //                         autoDrive.getMode() != DriveMode.INTAKE_IN_AUTO
        //                                 && autoDrive.getMode() != DriveMode.NONE)
        //         .whileTrue(superstructure.wristDown());
        mainController
                .leftBumper
                .and(() -> !superstructure.getPiece() && autoDrive.getMode() != DriveMode.CLEAN)
                .whileTrue(superstructure.intake().until(setPiece.and(isIntaking)));
        mainController
                .leftBumper
                .and(() -> !superstructure.getPiece() && autoDrive.getMode() == DriveMode.CLEAN)
                .whileTrue(
                        superstructure
                                .intake(() -> superstructure.getPiece())
                                .until(setPiece.and(isIntaking)));
        // .whileTrue(superstructure.pivotCommands.setAngle(() -> 20));
        setPiece.and(isIntaking)
                .and(() -> autoDrive.getMode() != DriveMode.CLEAN)
                .onTrue(superstructure.setPiece())
                .onTrue(superstructure.passToShooter());
        setPiece.and(isIntaking)
                .and(() -> autoDrive.getMode() == DriveMode.CLEAN)
                .onTrue(superstructure.cleanShoot())
                .onTrue(superstructure.passToShooterClean());
        mainController
                .leftBumper
                .or(() -> superstructure.getPiece())
                .onFalse(superstructure.stowIntake())
                .onFalse(superstructure.kickerCommands.kill());
        mainController
                .leftBumper
                .and(() -> detector.hasTarget() && autoDrive.getMode() != DriveMode.CLEAN)
                .onFalse(autoDrive.setMode(DriveMode.NONE));

        mainController.buttonA.onTrue(superstructure.ejectPiece());
        mainController.buttonA.onTrue(superstructure.setIsNotIntaking());

        coController.dPadLeft.onTrue(targetingUtil.offsetUp());
        coController.dPadRight.onTrue(targetingUtil.offsetDown());

        mainController.dPadUp.and(goodToClimb).whileTrue(climbCommands.goUp());
        mainController.dPadUp.onFalse(climbCommands.kill());
        mainController.dPadDown.and(goodToClimb).whileTrue(climbCommands.goDown());
        mainController.dPadDown.onFalse(climbCommands.kill());

        climbing.onTrue(superstructure.setIsClimbing());
        climbing.onFalse(superstructure.setIsNotClimbing());

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
        climbLeft.setDefaultCommand(climbCommands.holdLeftPositionAtCall());
        climbRight.setDefaultCommand(climbCommands.holdRightPositionAtCall());
        pivot.setDefaultCommand(superstructure.prep());
        intake.setDefaultCommand(superstructure.intakeCommands.kill());
        kicker.setDefaultCommand(superstructure.kickerCommands.kill());
        churro.setDefaultCommand(superstructure.churroCommands.holdPositionAtCall());
        wrist.setDefaultCommand(superstructure.wristCommands.holdPositionAtCall());
        shooterRight.setDefaultCommand(superstructure.shooterCommands.killRight());
        shooterLeft.setDefaultCommand(superstructure.shooterCommands.killLeft());
    }

    public void teleopInit() {}

    void configTestCommands() {}

    public void configureSmartDashboardLogging() {
        printer.addBoolean("note seen", () -> !autoCommands.noteNotSeen.getAsBoolean());
        // SmartDashboard.putNumber("pivot output", 0);
        // // SmartDashboard.putNumber("wrist kg", 0);
        // // printer.addDouble("wanted pivot", superstructure::getWantedPivot);
        // printer.addDouble("back tof", pivot::tofBack);
        // printer.addBoolean("front tof", pivot::frontPiece);
        printer.addDouble("front tof", pivot::tofFront);
        printer.addDouble("lef pos", climbLeft::getPosition);
        printer.addDouble("right pos", climbRight::getPosition);
        // // printer.addDouble("wrist", wrist::getAngle);
        // printer.addDouble("pivot", pivot::getAngle);
        // printer.addDouble("churo", churro::getAngle);
        // SmartDashboard.putNumber("bill", 3.93);
        // // printer.addBoolean("under stage", swerve::underStage);
        // // printer.addBoolean("set piece", () -> setPiece.getAsBoolean());
        // // printer.addBoolean("swerve aimed", autoDrive::swerveAimed);
        // // printer.addBoolean("spun up", superstructure::flywheelReadY);
        // // printer.addBoolean("pviot ready", superstructure::pivotReady);
        // // printer.addDouble("flywheel speed", shooterLeft::getVelocity);
        SmartDashboard.putNumber("pivot interp angle", 40);
        SmartDashboard.putNumber("shoot speed left", 15);
        SmartDashboard.putNumber("shoot speed right", 15);
        // SmartDashboard.putNumber("shoot speed", 15);
        // SmartDashboard.putNumber("differential", 1.1);
        printer.addDouble("shooter distance", targetingUtil::distance);
        // printer.addBoolean("current sensing", () -> autoCommands.currentYes.getAsBoolean());
        // printer.addBoolean("wrist at stow", superstructure::wristAtStow);
        // printer.addDouble("climb len", () -> climb.getLength());
        // printer.addBoolean("shooter ready", superstructure::flywheelReadY);
        // printer.addBoolean("pivot ready", superstructure::pivotReady);
        // printer.addBoolean("swerve ready", superstructure::swerveReady);
        // printer.addDouble("tx", detector::getTX);
        SmartDashboard.putNumber("left", 28);
        SmartDashboard.putNumber("right", 18);
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
                    ShooterConstants.kRightSlave,
                    ShooterConstants.kNativeVelToSurfaceMpS,
                    1,
                    ShooterConstants.kNominalVoltage,
                    0.02,
                    ShooterConstants.ff);

    public final ShooterLeft shooterLeft =
            new ShooterLeft(
                    ShooterConstants.kLeftRoller,
                    ShooterConstants.kLeftSlave,
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

    private final ClimbLeft climbLeft =
            new ClimbLeft(
                    ClimbConstants.kLeft,
                    1,
                    1,
                    ClimbConstants.kMinLength,
                    ClimbConstants.kMaxDegreeLength,
                    ClimbConstants.nominalVoltage,
                    0,
                    0.02);

    private final ClimbRight climbRight =
            new ClimbRight(
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

    private final ClimbCommands climbCommands = new ClimbCommands(climbLeft, climbRight);

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
                    right,
                    left,
                    backLeft,
                    backRight,
                    zoom);

    public final NeuralDetector detector = new NeuralDetectorPhotonVision(VisionConstants.driver);

    public final RobotTracker tracker =
            new RobotTracker(
                    FieldConstants.kBlueSpeaker,
                    FieldConstants.kBlueAmp,
                    FieldConstants.kBlueFeed,
                    PivotConstants.robotToPivot2d,
                    swerve::getOdoPose,
                    swerve::getChassisSpeeds,
                    ShooterConstants.kLeftMap,
                    true);

    public final TargetingUtil targetingUtil = new TargetingUtil(tracker);

    public final AutoDrive autoDrive =
            new AutoDrive(
                    swerve,
                    detector,
                    targetingUtil,
                    ShooterConstants.kLeftMap,
                    ShooterConstants.kRightMap,
                    ShooterConstants.kFeedMap);
    public final Superstructure superstructure =
            new Superstructure(
                    intake,
                    kicker,
                    shooterRight,
                    shooterLeft,
                    pivot,
                    wrist,
                    churro,
                    autoDrive::getPivotAngle,
                    autoDrive::getShootSpeedLeft,
                    autoDrive::getShootSpeedRight,
                    autoDrive::flywheelThreshold,
                    targetingUtil.exitVelocity(),
                    autoDrive::isFeed,
                    autoDrive::swerveAimed);

    LEDTriggers ledTriggers = new LEDTriggers(superstructure, autoDrive::getMode);

    private final team3647.frc2024.subsystems.LEDs LEDs =
            new team3647.frc2024.subsystems.LEDs(LEDConstants.m_candle, ledTriggers);

    public final AutoCommands autoCommands =
            new AutoCommands(
                    swerve,
                    autoDrive::setMode,
                    autoDrive::getVelocities,
                    autoDrive::onTheMove,
                    autoDrive::getDriveRotAmp,
                    autoDrive::getDriveRotOther90,
                    autoDrive::getDriveXCenter,
                    superstructure,
                    targetingUtil,
                    detector::hasTarget,
                    autoDrive::getMode);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    final GroupPrinter printer = GroupPrinter.getInstance();

    private final Trigger piece = new Trigger(() -> superstructure.getPiece());

    private final Trigger climbing =
            new Trigger(() -> climbLeft.getPosition() > 1 && climbRight.getPosition() > 1);

    private final Trigger goodToAmp =
            new Trigger(
                    () ->
                            !coController.buttonY.getAsBoolean()
                                    && climbLeft.getPosition() < 5
                                    && climbRight.getPosition() < 5);

    private final Trigger goodToClimb =
            new Trigger(() -> autoDrive.getMode() != DriveMode.SHOOT_AT_AMP);

    private final Trigger setPiece =
            new Trigger(() -> superstructure.current() && wrist.getAngle() < 5) // 41
                    .debounce(0.06)
                    .or(mainController.buttonB);

    private final BooleanSupplier isIntaking =
            () -> mainController.leftBumper.getAsBoolean() && !DriverStation.isAutonomous();
}
