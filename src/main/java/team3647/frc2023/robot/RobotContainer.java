package team3647.frc2023.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3647.frc2023.auto.AutoCommands;
import team3647.frc2023.auto.AutonomousMode;
import team3647.frc2023.commands.DrivetrainCommands;
import team3647.frc2023.constants.FieldConstants;
// import team3647.frc2023.auto.AutoCommands;
// // import team3647.frc2023.auto.AutoCommands;
// import team3647.frc2023.auto.AutonomousMode;
import team3647.frc2023.constants.GlobalConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.constants.VisionConstants;
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
    private final SendableChooser<Command> autoChooser;

    //     private Twist2d twist = new Twist2d();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // if driving is bill, revert drivetraincommands, swervedrive, swervedriveconstants, and
        // robotcontainer
        pdh.clearStickyFaults();
        scheduler.registerSubsystem(swerve);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        autoCommands.registerCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("auto", autoChooser);
        runningMode =
                new AutonomousMode(
                        new InstantCommand(),
                        kEmptyPose); // currently using this, not the auto chooser
        swerve.setRobotPose(new Pose2d(1.37, 5.54, new Rotation2d()));
        // add the robot to cam transforms
    }

    private void configureButtonBindings() {

        mainController.rightBumper.onTrue(autoDrive.setMode(DriveMode.SHOOT_STATIONARY));
        mainController.rightBumper.onFalse(autoDrive.setMode(DriveMode.NONE));
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
                        autoDrive::getVelocities));
        // shooter.setDefaultCommand(shooterCommands.shoot(mainController::getLeftTriggerValue));

        // wrist.setDefaultCommand(superstructure.wristCommands.holdPositionAtCall());
    }

    public void teleopInit() {}

    void configTestCommands() {
        // Commands.run(() -> {}, grabber).schedule();
    }

    public void configureSmartDashboardLogging() {
        // printer.addDouble("co right x", coController::getRightStickY);
        printer.addDouble("odo x", swerve::getPoseX);
        printer.addDouble("odo y", swerve::getPoseY);
        // printer.addPose("psoe", swerve);
        // printer.addDouble("drive speed", swerve::getAverageSpeed);
        // printer.addDouble("wrist", cubeWrist::getAngle);
    }

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

    // public final Shooter shooter =
    //         new Shooter(
    //                 team3647.frc2023.constants.ShooterConstants.kTopRoller,
    //                 team3647.frc2023.constants.ShooterConstants.kBottomRoller,
    //                 1,
    //                 1,
    //                 team3647.frc2023.constants.ShooterConstants.kNominalVoltage,
    //                 0.02);

    // public final ShooterCommands shooterCommands = new ShooterCommands(shooter);

    private final Compressor compressor = new Compressor(GlobalConstants.kPCMType);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final DrivetrainCommands drivetrainCommands = new DrivetrainCommands(swerve);

    private VisionController visionController =
            new VisionController(swerve::addVisionData, swerve, VisionConstants.ar_doo_cam);
    // () -> twist);

    public final AutoCommands autoCommands = new AutoCommands(swerve);

    public final NeuralDetector detector = new NeuralDetector(VisionConstants.limelightName);

    public final AutoDrive autoDrive =
            new AutoDrive(
                    swerve,
                    detector,
                    new TargetingUtil(FieldConstants.kBlueSpeaker, swerve::getOdoPose));

    //     public final AutoCommands autoCommands =
    //             new AutoCommands(swerve, SwerveDriveConstants.kDriveKinematics, superstructure);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    final GroupPrinter printer = GroupPrinter.getInstance();
    // private final AutoCommands autoCommands =
    // new AutoCommands(swerve, SwerveDriveConstants.kDriveKinematics,
    // superstructure);

    private final Pose2d kEmptyPose = new Pose2d();

    // private final Trigger coControllerRightJoystickMoved =
    // new Trigger(() -> Math.abs(coController.getRightStickY()) >= 0.2);
}
