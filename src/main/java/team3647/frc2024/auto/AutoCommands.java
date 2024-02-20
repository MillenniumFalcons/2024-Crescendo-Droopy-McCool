package team3647.frc2024.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import team3647.frc2024.constants.AutoConstants;
import team3647.frc2024.constants.FieldConstants;
import team3647.frc2024.subsystems.Superstructure;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.frc2024.util.TargetingUtil;

public class AutoCommands {
    private final SwerveDrive swerve;
    private final Supplier<Twist2d> autoDriveVelocities;
    private final Superstructure superstructure;
    private final TargetingUtil targeting;

    private final String s1_to_n1 = "s1 to n1";
    private final String s1_to_n1_to_f1 = "s1 to n1 to f1";
    private final String s2_to_n2 = "s2 to n2";
    private final String n2_to_n1 = "n2 to n1";
    private final String n2_to_n3 = "n2 to n3";
    private final String s3_to_n3 = "s3 to n3";
    private final String n3_to_f5 = "n3 to f5";
    private final String n1_to_n2 = "n1 to n2";
    private final String n3_to_n2 = "n3 to n2";
    private final String f1_to_n2 = "f1 to n2";
    private final String f1_to_shoot1 = "f1 to shoot1";
    private final String shoot1_to_f2 = "shoot1 to f2";
    private final String f2_to_n2 = "f2 to n2";
    private final String n2_to_f1 = "n2 to f1";
    private final String f2_to_shoot1 = "f2 to shoot1";
    private final String shoot1_to_f3 = "shoot1 to f3";
    private final String f3_to_shoot2 = "f3 to shoot2";
    private final String f5_to_shoot3 = "f5 to shoot3";
    private final String shoot3_to_f4 = "shoot3 to f4";
    private final String f4_to_shoot3 = "f4 to shoot3";
    private final String shoot3_to_f3 = "shoot3 to f3";
    private final String s3_to_f5 = "s3 to f5";

    public final Trigger currentYes;

    public final AutonomousMode blueFive_S1N1F1N2N3;

    public final AutonomousMode blueFour_S1N1N2N3;

    public final AutonomousMode yes;

    public AutoCommands(
            SwerveDrive swerve,
            Supplier<Twist2d> autoDriveVelocities,
            Superstructure superstructure,
            TargetingUtil targeting) {
        this.swerve = swerve;
        this.autoDriveVelocities = autoDriveVelocities;
        this.superstructure = superstructure;
        this.targeting = targeting;

        currentYes = new Trigger(() -> superstructure.currentYes()).debounce(0.06);

        this.yes = new AutonomousMode(four_S3N5N4N3(Alliance.Blue), getInitial(s3_to_f5));

        this.blueFive_S1N1F1N2N3 =
                new AutonomousMode(five_S1N1F1N2N3(Alliance.Blue), getInitial(s1_to_n1_to_f1));

        this.blueFour_S1N1N2N3 =
                new AutonomousMode(four_S1N1N2N3(Alliance.Blue), getInitial(s1_to_n1));
    }

    public void registerCommands() {}

    public AutonomousMode getSix_S1N1F1N2N3ByColor(Alliance color) {
        return new AutonomousMode(six_S1N1F1F2N2N3(color), getInitial(s1_to_n1_to_f1));
    }

    public AutonomousMode getFive_S1N1F1N2N3ByColor(Alliance color) {
        return new AutonomousMode(five_S1N1F1N2N3(color), getInitial(s1_to_n1_to_f1));
    }

    public AutonomousMode getFour_S1N1F1N2N3ByColor(Alliance color) {
        return new AutonomousMode(four_S1N1N2N3(color), getInitial(s1_to_n1));
    }

    public Command six_S1N1F1F2N2N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(),
                Commands.sequence(
                        followChoreoPathWithOverride(s1_to_n1_to_f1, color),
                        followChoreoPathWithOverride(f1_to_shoot1, color),
                        followChoreoPathWithOverride(shoot1_to_f2, color),
                        followChoreoPathWithOverride(f2_to_n2, color),
                        followChoreoPathWithOverride(n2_to_n3, color)));
    }

    public Command four_S3N5N4N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(),
                Commands.sequence(
                        followChoreoPathWithOverride(s3_to_f5, color),
                        followChoreoPathWithOverride(f5_to_shoot3, color),
                        followChoreoPathWithOverride(shoot3_to_f4, color),
                        followChoreoPathWithOverride(f4_to_shoot3, color),
                        followChoreoPathWithOverride(shoot3_to_f3, color),
                        followChoreoPathWithOverride(f3_to_shoot2, color)));
    }

    public Command five_S1N1F1N2N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(),
                Commands.sequence(
                        followChoreoPathWithOverrideFast(s1_to_n1_to_f1, color),
                        followChoreoPathWithOverrideFast(f1_to_n2, color),
                        followChoreoPathWithOverride(n2_to_n3, color)));
    }

    public Command four_S1N1N2N3(Alliance color) {
        return Commands.parallel(
                masterSuperstructureSequence(),
                Commands.sequence(
                        followChoreoPathWithOverride(s1_to_n1, color),
                        followChoreoPathWithOverride(n1_to_n2, color),
                        followChoreoPathWithOverride(n2_to_n3, color)));
    }

    public Command masterSuperstructureSequence() {
        return Commands.sequence(
                scorePreload(),
                Commands.parallel(
                        superstructure.spinUp(),
                        superstructure.prep(),
                        // superstructure.fastFeed(),
                        continuouslyIntakeForShoot().repeatedly(),
                        superstructure.autoFeed(() -> goodToGo())));
    }

    public boolean goodToGo() {
        return swerve.getOdoPose().getX() < AutoConstants.kDrivetrainXShootingThreshold;
    }

    public Command scorePreload() {
        return Commands.parallel(
                        superstructure.spinUp(),
                        superstructure.prep(),
                        Commands.sequence(Commands.waitSeconds(0.3), superstructure.feed()))
                .withTimeout(0.6);
    }

    public Pose2d getInitial(String path) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        return traj.getInitialPose();
    }

    public Pose2d getFinal(String path) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        return traj.getFinalPose();
    }

    public Command spinUp() {
        return Commands.parallel(superstructure.spinUp(), superstructure.feed());
    }

    public double getPivotAngleByPose(Supplier<Pose2d> pose) {
        return targeting.getPivotAngleByPose(pose.get()) * 180 / Math.PI;
    }

    public double getPivotAngle() {
        return targeting.getPivotAngle(FieldConstants.kBlueSpeaker) * 180 / Math.PI;
    }

    public Command target() {
        return Commands.run(() -> swerve.drive(0, 0, deeThetaOnTheMove()), swerve).withTimeout(2);
    }

    public Command shoot() {
        return Commands.parallel(superstructure.shootStow());
    }

    public Command continuouslyIntakeForShoot() {
        return Commands.sequence(
                superstructure
                        .intake()
                        .until(currentYes)
                        .andThen(superstructure.passToShooterNoKicker()));
    }

    public Pose2d flipForPP(Pose2d pose) {
        return new Pose2d(
                new Translation2d(FieldConstants.kFieldLength - pose.getX(), pose.getY()),
                pose.getRotation().rotateBy(FieldConstants.kOneEighty));
    }

    public Command overrideChoreoPathWithIntake(String path, Alliance color) {
        return Commands.deadline(pathAndShootWithOverride(path, color), superstructure.intake());
    }

    public Command pathAndShootWithOverride(String path, Alliance color) {
        return Commands.parallel(
                superstructure.shootStow(), followChoreoPathWithOverride(path, color));
    }

    public Command followChoreoPathWithOverrideAndPivot(String path, Alliance color) {
        return Commands.deadline(
                followChoreoPathWithOverride(path, color),
                superstructure.pivotCommands.setAngle(() -> getPivotAngle()));
    }

    public Command followChoreoPathWithOverrideFast(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        return customChoreoFolloweForOverride(
                traj,
                swerve::getOdoPose,
                choreoSwerveController(
                        AutoConstants.kXController,
                        AutoConstants.kYController,
                        new PIDController(0, 0, 0)),
                (ChassisSpeeds speeds) ->
                        swerve.drive(
                                speeds.vxMetersPerSecond * 0.7,
                                speeds.vyMetersPerSecond * 0.7,
                                deeThetaOnTheMove()),
                () -> mirror);
    }

    public Command followChoreoPathWithOverride(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        return customChoreoFolloweForOverride(
                traj,
                swerve::getOdoPose,
                choreoSwerveController(
                        AutoConstants.kXController,
                        AutoConstants.kYController,
                        new PIDController(0, 0, 0)),
                (ChassisSpeeds speeds) ->
                        swerve.drive(
                                speeds.vxMetersPerSecond * 0.4,
                                speeds.vyMetersPerSecond * 0.4,
                                deeThetaOnTheMove()),
                () -> mirror);
    }

    public Command customChoreoFolloweForOverride(
            ChoreoTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            ChoreoControlFunction controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            BooleanSupplier mirrorTrajectory) {
        var timer = new edu.wpi.first.wpilibj.Timer();
        return new FunctionalCommand(
                timer::restart,
                () -> {
                    ;
                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    trajectory.sample(
                                            timer.get(), mirrorTrajectory.getAsBoolean())));
                },
                (interrupted) -> {
                    timer.stop();
                    outputChassisSpeeds.accept(new ChassisSpeeds());
                },
                () -> timer.hasElapsed(0.5) && swerve.getVel() < 0.1,
                swerve);
    }

    public static ChoreoControlFunction choreoSwerveController(
            PIDController xController,
            PIDController yController,
            PIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double rotationFF = referenceState.angularVelocity;

            double xFeedback = xController.calculate(pose.getX(), referenceState.x);
            double yFeedback = yController.calculate(pose.getY(), referenceState.y);
            double rotationFeedback =
                    rotationController.calculate(
                            pose.getRotation().getRadians(), referenceState.heading);

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback,
                    yFF + yFeedback,
                    rotationFF + rotationFeedback,
                    pose.getRotation());
        };
    }

    public double deeThetaOnTheMove() {
        return autoDriveVelocities.get().dtheta;
    }

    public Command followChoreoPath(String path, Alliance color) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        boolean mirror = color == Alliance.Red;
        return Choreo.choreoSwerveCommand(
                traj,
                swerve::getOdoPose,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                (ChassisSpeeds speeds) ->
                        swerve.drive(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond,
                                speeds.omegaRadiansPerSecond),
                () -> mirror,
                swerve);
    }
}
