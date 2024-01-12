package team3647.frc2023.util;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.lib.team6328.VirtualSubsystem;

public class AutoDrive extends VirtualSubsystem {
    private SwerveDrive swerve;
    private NeuralDetector detector;
    private TargetingUtil targeting;
    private Pose2d targetPose = new Pose2d();
    private DriveMode mode = DriveMode.NONE;
    private double targetRot = 0;
    private boolean enabled = true;

    private final ProfiledPIDController rotController =
            new ProfiledPIDController(
                    1,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(1, 2)); // new PIDController(0.4, 0, 0);

    private final PIDController quickRotController =
            new PIDController(4, 0, 0); // new PIDController(0.4, 0, 0);

    private final PIDController quickerRotController =
            new PIDController(6, 0, 0); // new PIDController(0.4, 0, 0);

    private final ProfiledPIDController xController =
            new ProfiledPIDController(2.5, 0, 0, new TrapezoidProfile.Constraints(2, 3));
    private final ProfiledPIDController yController =
            new ProfiledPIDController(2.5, 0, 0, new TrapezoidProfile.Constraints(2, 3));

    public AutoDrive(SwerveDrive swerve, NeuralDetector detector, TargetingUtil targeting) {
        this.detector = detector;
        this.swerve = swerve;
        this.targeting = targeting;
    }

    public enum DriveMode {
        INTAKE_FLOOR_PIECE,
        ALIGN_TO_AMP,
        SHOOT_ON_THE_MOVE,
        SHOOT_STATIONARY,
        NONE
    }

    @Override
    public void periodic() {
        if (this.mode == DriveMode.SHOOT_STATIONARY) {
            targetRot = targeting.angleToSpeaker();
        } else if (this.mode == DriveMode.SHOOT_ON_THE_MOVE) {
            targetRot = targeting.angleToSpeakerOnTheMove();
        }
        var bill = detector.pieceCoordinate(swerve::getOdoPose);
        if (bill.isPresent()) {
            targetPose = bill.get();
            swerve.field.getObject("piece location").setPose(targetPose);
        }
    }

    public Command enable() {
        return Commands.runOnce(() -> this.enabled = true);
    }

    public Command disable() {
        return Commands.runOnce(() -> this.enabled = false);
    }

    public boolean getEnabled() {
        return this.enabled;
    }

    public Command setMode(DriveMode mode) {
        return Commands.runOnce(() -> this.mode = mode);
    }

    public DriveMode getMode() {
        return this.mode;
    }

    public double getPivotAngle() {
        switch (mode) {
            case SHOOT_ON_THE_MOVE:
                return targeting.getPivotAngleByDistanceOnTheMove();
            case SHOOT_STATIONARY:
                return targeting.getPivotAngleByDistance();
            default:
                return 90;
        }
    }

    public double getX() {
        xController.setGoal(targetPose.getX());
        double k = xController.calculate(swerve.getOdoPose().getX());
        double setpoint = Math.abs(k) < 0.15 ? 0 : k;
        return setpoint;
    }

    public double getY() {
        yController.setGoal(targetPose.getY());
        double k = yController.calculate(swerve.getOdoPose().getY());
        double setpoint = Math.abs(k) < 0.15 ? 0 : k;
        return setpoint;
    }

    public double getRot() {
        rotController.setGoal(targetPose.getRotation().getRadians());
        double k = rotController.calculate(swerve.getOdoPose().getRotation().getRadians());
        k = quickRotController.calculate(targetRot, 0);
        double setpoint = Math.abs(k) < 0.03 ? 0 : k;
        return setpoint;
    }

    public Twist2d getVelocities() {
        return new Twist2d(getX(), getY(), getRot());
    }

    public PathPlannerPath getPathToAmp() {
        return new PathPlannerPath(
                null,
                AutoConstants.defaultConstraints,
                new GoalEndState(0, new Rotation2d(-Math.PI / 2)));
    }

    public PathPlannerPath getPathToClosestPiece() {
        return new PathPlannerPath(
                null,
                AutoConstants.defaultConstraints,
                new GoalEndState(0, new Rotation2d(-Math.PI / 2)));
    }

    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathWithEvents(
                new FollowPathHolonomic(
                        path,
                        swerve::getOdoPose,
                        swerve::getChassisSpeeds,
                        swerve::drive,
                        new HolonomicPathFollowerConfig(
                                AutoConstants.kTranslationConstants,
                                AutoConstants.kRotationConstants,
                                6,
                                SwerveDriveConstants.kTrackWidth / 2.0 * Math.sqrt(2.0),
                                new ReplanningConfig()),
                        swerve),
                path,
                swerve::getOdoPose);
    }
}
