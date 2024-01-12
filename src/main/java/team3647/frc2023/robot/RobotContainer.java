package team3647.frc2023.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.constants.VisionConstants;
import team3647.frc2023.subsystems.Intake;
import team3647.frc2023.subsystems.Kicker;
import team3647.frc2023.subsystems.Pivot;
import team3647.frc2023.subsystems.Shooter;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
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
    // private final Shooter shooter;

    //     private Twist2d twist = new Twist2d();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // if driving is bill, revert drivetraincommands, swervedrive, swervedriveconstants, and
        // robotcontainer
        pdh.clearStickyFaults();
        scheduler.registerSubsystem(swerve, shooter, intake, kicker, pivot);

        // shooter =
        //         new Shooter(
        //                 ShooterConstants.kBottomRoller,
        //                 ShooterConstants.kTopRoller,
        //                 1,
        //                 1,
        //                 12,
        //                 0.02);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        autoCommands.registerCommands();
        runningMode = autoCommands.blueFour_S1N1N2N3; // currently using this, not the auto chooser
        swerve.setRobotPose(runningMode.getPathplannerPose2d());
        // add the robot to cam transforms
    }

    private void configureButtonBindings() {

        mainController.leftTrigger.whileTrue(autoDrive.setMode(DriveMode.INTAKE_FLOOR_PIECE));
        mainController.rightTrigger.whileTrue(autoDrive.setMode(DriveMode.ALIGN_TO_AMP));
        mainController.leftBumper.whileTrue(autoDrive.setMode(DriveMode.SHOOT_ON_THE_MOVE));
        mainController.rightBumper.whileTrue(autoDrive.setMode(DriveMode.SHOOT_STATIONARY));
        mainController.rightBumper.onFalse(autoDrive.setMode(DriveMode.NONE));
        mainController.leftBumper.onFalse(autoDrive.setMode(DriveMode.NONE));
        mainController.leftTrigger.onFalse(autoDrive.setMode(DriveMode.NONE));
        mainController.rightTrigger.onFalse(autoDrive.setMode(DriveMode.NONE));

        mainController.leftBumper.onTrue(superstructure.shoot());
        mainController.rightBumper.onTrue(superstructure.shoot());

        mainController.leftBumper.onFalse(superstructure.stow());
        mainController.rightBumper.onFalse(superstructure.stow());

        mainController
                .leftTrigger
                .whileTrue(superstructure.intake())
                .onFalse(superstructure.stow());

        // need to change this to a conditional command so it doesn't start auto aiming
        // when doing
        // cubes from cube shooter

        // mainController.buttonY.onTrue(new InstantCommand(() ->
        // swerve.resetModuleAngle()));
        // mainController.leftBumper.whileTrue(autoDrive.driveToPose());

        // mainController
        //         .leftBumper
        //         .and(() -> autoDrive.getIsGoodToIntake())
        //         .whileTrue(superstructure.cubeShooterIntake())
        //         .onFalse(superstructure.cubeShooterCommands.stow());

        // mainController.leftTrigger.whileTrue(superstructure.cubeShooterIntake());
        // mainController.leftTrigger.onFalse(superstructure.cubeShooterCommands.stow());
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                drivetrainCommands.driveVisionTeleop(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        () -> false,
                        () -> true,
                        // enable autosteer if going to actual station (bumper), or scoring
                        // (trigger)
                        // () -> false,
                        autoDrive::getMode,
                        autoDrive::getEnabled,
                        autoDrive::getVelocities));
        pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());
        // shooter.setDefaultCommand(shooterCommands.shoot(mainController::getLeftTriggerValue));

        // wrist.setDefaultCommand(superstructure.wristCommands.holdPositionAtCall());
    }

    public void teleopInit() {}

    void configTestCommands() {}

    public void configureSmartDashboardLogging() {}

    // counted relative to what driver sees
    public Command getAutonomousCommand() {
        return runningMode.getAutoCommand();
        // return autoChooser.getSelected();
    }

    private final Joysticks mainController = new Joysticks(0);
    // private final Joysticks coController = new Joysticks(1);
    public final SwerveDrive swerve =
            new SwerveDrive(
                    SwerveDriveConstants.kFrontLeftModule,
                    SwerveDriveConstants.kFrontRightModule,
                    SwerveDriveConstants.kBackLeftModule,
                    SwerveDriveConstants.kBackRightModule,
                    SwerveDriveConstants.kGyro,
                    SwerveDriveConstants.kDriveKinematics,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
                    GlobalConstants.kDt);

    public final Shooter shooter =
            new Shooter(
                    team3647.frc2023.constants.ShooterConstants.kTopRoller,
                    team3647.frc2023.constants.ShooterConstants.kBottomRoller,
                    1,
                    1,
                    team3647.frc2023.constants.ShooterConstants.kNominalVoltage,
                    0.02);

    public final Kicker kicker =
            new Kicker(KickerConstants.kMaster, 1, 1, KickerConstants.kNominalVoltage, 0.02);

    public final Intake intake =
            new Intake(IntakeConstants.kMaster, 1, 1, IntakeConstants.kNominalVoltage, 0.02);

    public final Pivot pivot =
            new Pivot(
                    PivotConstants.kMaster,
                    PivotConstants.kNativeVelToDPS,
                    PivotConstants.kNativePosToDegrees,
                    PivotConstants.kMinDegree,
                    PivotConstants.kMaxDegree,
                    PivotConstants.nominalVoltage,
                    0.02);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final DrivetrainCommands drivetrainCommands = new DrivetrainCommands(swerve);

    private VisionController visionController =
            new VisionController(swerve::addVisionData, swerve, VisionConstants.ar_doo_cam);
    // () -> twist);

    public final AutoCommands autoCommands = new AutoCommands(swerve);

    public final NeuralDetector detector = new NeuralDetector(VisionConstants.limelightName);

    public final TargetingUtil targetingUtil =
            new TargetingUtil(
                    FieldConstants.kBlueSpeaker,
                    FieldConstants.kSpeakerHeight,
                    swerve::getOdoPose,
                    swerve::getFieldRelativeChassisSpeeds,
                    PivotConstants.robotToPivot);

    public final AutoDrive autoDrive = new AutoDrive(swerve, detector, targetingUtil);

    public final Superstructure superstructure =
            new Superstructure(intake, kicker, shooter, pivot, autoDrive::getPivotAngle);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    final GroupPrinter printer = GroupPrinter.getInstance();
}
