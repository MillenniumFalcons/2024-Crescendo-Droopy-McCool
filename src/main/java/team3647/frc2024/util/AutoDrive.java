package team3647.frc2024.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import team3647.frc2024.constants.AutoConstants;
import team3647.frc2024.constants.FieldConstants;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.lib.team6328.VirtualSubsystem;

public class AutoDrive extends VirtualSubsystem {
    private final SwerveDrive swerve;
    private final NeuralDetector detector;
    private final TargetingUtil targeting;
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
            new PIDController(6, 0, 0.5); // new PIDController(0.4, 0, 0);

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
        SHOOT_AT_AMP,
        SHOOT_ON_THE_MOVE,
        NONE
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Robot/Compensated", targeting.compensatedPose());
        if (this.mode == DriveMode.SHOOT_ON_THE_MOVE || DriverStation.isAutonomous()) {
            targetRot = targeting.shootAtSpeaker().rotation;
        }
        if (this.mode == DriveMode.INTAKE_FLOOR_PIECE) {
            targetRot = -detector.getTX();
        }
        if (this.mode == DriveMode.SHOOT_AT_AMP) {
            targetRot = targeting.shootAtAmp().rotation;
        }
        SmartDashboard.putNumber("auto drive", getRot());
    }

    public boolean swerveAimed() {
        return getRot() < 0.3;
    }

    private void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        swerve.field.getObject("piece pose").setPose(targetPose);
        xController.setGoal(targetPose.getX());
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

    public double getShootSpeed() {
        if (DriverStation.isAutonomous()) {
            return targeting.shootAtSpeaker().shootSpeed;
        }
        switch (mode) {
            case SHOOT_ON_THE_MOVE:
                return targeting.shootAtSpeaker().shootSpeed;
            case SHOOT_AT_AMP:
                return targeting.shootAtAmp().shootSpeed;
            default:
                return 25;
        }
    }

    public double getPivotAngle() {
        if (DriverStation.isAutonomous()) {
            return Units.radiansToDegrees(targeting.shootAtSpeaker().pivot);
        }
        switch (mode) {
            case SHOOT_ON_THE_MOVE:
                return Units.radiansToDegrees(targeting.shootAtSpeaker().pivot);
            case SHOOT_AT_AMP:
                return Units.radiansToDegrees(targeting.shootAtAmp().pivot);
            default:
                return Units.radiansToDegrees(targeting.shootAtSpeaker().pivot);
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
        var pose = swerve.getOdoPose();
        var endHolonomicRotation = new Rotation2d(-Math.PI / 2);
        var vector =
                VecBuilder.fill(
                        FieldConstants.kBlueAmpAlign.getX() - pose.getX(),
                        FieldConstants.kBlueAmpAlign.getY() - pose.getY());
        var rotation = Math.acos(vector.dot(VecBuilder.fill(1, 0)) / vector.norm());
        Pose2d startingPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(rotation));
        List<Translation2d> bezierPoints =
                PathPlannerPath.bezierFromPoses(startingPose, FieldConstants.kBlueAmpAlign);
        return new PathPlannerPath(
                bezierPoints,
                AutoConstants.defaultConstraints,
                new GoalEndState(0, endHolonomicRotation));
    }
}
