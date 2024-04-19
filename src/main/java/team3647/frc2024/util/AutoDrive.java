package team3647.frc2024.util;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.Supplier;
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

    private InterpolatingDoubleTreeMap shootSpeedMapLeft;
    private InterpolatingDoubleTreeMap shootSpeedMapRight;
    private InterpolatingDoubleTreeMap feedMap;

    private final ProfiledPIDController rotController =
            new ProfiledPIDController(
                    6,
                    0,
                    0.3,
                    new TrapezoidProfile.Constraints(10, 10)); // new PIDController(0.4, 0, 0);

    // private final PIDController rotController =
    //         new PIDController(6, 0, 0); // new PIDController(0.4, 0, 0);

    private final PIDController quickerRotController =
            new PIDController(12, 0, 1.2); // new PIDController(0.4, 0, 0);

    private final PIDController xController =
            new PIDController(1, 0, 0); // new PIDController(0.4, 0, 0);

    private final ProfiledPIDController fastXController =
            new ProfiledPIDController(2, 0, 0.3, new TrapezoidProfile.Constraints(5, 10));

    private final PIDController fasterXController =
            new PIDController(3, 0, 0); // new PIDController(0.4, 0, 0);

    private final ProfiledPIDController fastYController =
            new ProfiledPIDController(
                    2,
                    0,
                    0.3,
                    new TrapezoidProfile.Constraints(5, 10)); // new PIDController(0.4, 0, 0);

    private final PIDController yController =
            new PIDController(5, 0, 0); // new PIDController(0.4, 0, 0);

    public AutoDrive(
            SwerveDrive swerve,
            NeuralDetector detector,
            TargetingUtil targeting,
            InterpolatingDoubleTreeMap shootSpeedMapLeft,
            InterpolatingDoubleTreeMap shootSpeedMapRight,
            InterpolatingDoubleTreeMap feedSpeedMap) {
        this.detector = detector;
        this.swerve = swerve;
        this.targeting = targeting;
        this.shootSpeedMapLeft = shootSpeedMapLeft;
        this.shootSpeedMapRight = shootSpeedMapRight;
        this.feedMap = feedSpeedMap;
        // rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public enum DriveMode {
        INTAKE_FLOOR_PIECE,
        SHOOT_AT_AMP,
        SHOOT_STATIONARY,
        SHOOT_ON_THE_MOVE,
        CLEAN,
        FEED,
        INTAKE_IN_AUTO,
        NONE
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("Robot/Compensated", targeting.32222getCompensatedPose());
        // Logger.recordOutput("Robot/Compensated", targeting.compensatedPose());
        SmartDashboard.putNumber("rot amo", targeting.rotToAmp());
        Logger.recordOutput("offset", targeting.getOffset());
        Logger.recordOutput("rot", targetRot);
        if (DriverStation.isAutonomous()) {
            targetRot = targeting.shootAtSpeakerOnTheMove().rotation;
        }
        if (this.mode == DriveMode.CLEAN) {
            targetRot = targeting.shootAtSpeakerOnTheMove().rotation;
        }
        if (this.mode == DriveMode.SHOOT_ON_THE_MOVE) {
            targetRot = targeting.shootAtSpeakerOnTheMove().rotation;
        }
        if (this.mode == DriveMode.FEED) {
            targetRot = targeting.shootAtFeed().rotation;
        }
        if (this.mode == DriveMode.SHOOT_STATIONARY) {
            targetRot = targeting.shootAtSpeaker().rotation;
        }
        if (this.mode == DriveMode.INTAKE_FLOOR_PIECE || this.mode == DriveMode.INTAKE_IN_AUTO) {
            targetRot = Units.degreesToRadians(detector.getTX());
        }
        if (this.mode == DriveMode.SHOOT_AT_AMP) {
            targetRot = targeting.shootAtAmp().rotation;
        }
        // rotController.setP(SmartDashboard.getNumber("kp", kp));
        // SmartDashboard.putNumber("auto drive", getRot());
    }

    public boolean swerveAimed() {
        return Math.abs(targetRot) < 0.035 * MathUtil.clamp(4 / targeting.distance(), 1, 10);
    }

    public double flywheelThreshold() {
        return getShootSpeedLeft() - (1 + MathUtil.clamp(8 - 2 * targeting.distance(), 0, 5));
    }

    private void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        // swerve.field.getObject("piece pose").setPose(targetPose);
        // xController.setGoal(targetPose.getX());
    }

    public boolean isFeed() {
        return this.mode == DriveMode.FEED;
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

    public Command setMode(Supplier<DriveMode> mode) {
        return Commands.run(() -> this.mode = mode.get());
    }

    public DriveMode getMode() {
        return this.mode;
    }

    public double getShootSpeedLeft() {
        if (DriverStation.isAutonomous()) {
            return shootSpeedMapLeft.get(targeting.getCompensatedDistance());
        }
        switch (mode) {
            case FEED:
                return feedMap.get(targeting.getCompensatedDistance());
            case SHOOT_STATIONARY:
                return shootSpeedMapLeft.get(targeting.distance());
            case SHOOT_ON_THE_MOVE:
                return shootSpeedMapLeft.get(targeting.getCompensatedDistance());
            default:
                return shootSpeedMapLeft.get(targeting.getCompensatedDistance());
        }
    }

    public double getShootSpeedRight() {
        if (DriverStation.isAutonomous()) {
            return shootSpeedMapRight.get(targeting.getCompensatedDistance());
        }
        switch (mode) {
            case FEED:
                return feedMap.get(targeting.getCompensatedDistance()) * 4 / 5;
            case SHOOT_STATIONARY:
                return shootSpeedMapRight.get(targeting.distance());
            case SHOOT_ON_THE_MOVE:
                return shootSpeedMapRight.get(targeting.getCompensatedDistance());
            default:
                return shootSpeedMapRight.get(targeting.getCompensatedDistance());
        }
    }

    public double getPivotAngle() {
        if (DriverStation.isAutonomous()) {
            return Units.radiansToDegrees(targeting.shootAtSpeakerOnTheMove().pivot);
        }
        switch (mode) {
            case FEED:
                return 45;
            case SHOOT_ON_THE_MOVE:
                return Units.radiansToDegrees(targeting.shootAtSpeakerOnTheMove().pivot);
            case SHOOT_STATIONARY:
                return Units.radiansToDegrees(targeting.shootAtSpeaker().pivot);
            case SHOOT_AT_AMP:
                return Units.radiansToDegrees(targeting.shootAtAmp().pivot);
            default:
                return Units.radiansToDegrees(targeting.shootAtSpeakerOnTheMove().pivot);
        }
    }

    public Pose2d getInitial(String path) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);
        return traj.getInitialPose();
    }

    public double getX() {
        if (this.mode == DriveMode.SHOOT_AT_AMP) {
            double k = fasterXController.calculate(targeting.pose().getX(), targeting.getAmpX());
            double setpoint = Math.abs(k) < 0.04 ? 0 : k;
            return setpoint;
        }
        double k = xController.calculate(-2, Units.degreesToRadians(detector.getTY()));
        double setpoint = Math.abs(k) < 0.02 ? 0 : k;
        return setpoint;
    }

    public double getY() {
        if (this.mode == DriveMode.SHOOT_AT_AMP) {
            double k = fastYController.calculate(targeting.pose().getY(), targeting.getAmpY());
            double setpoint = Math.abs(k) < 0.04 ? 0 : k;
            return setpoint;
        }
        double k = yController.calculate(Units.degreesToRadians(detector.getTX()), 0);
        double setpoint = Math.abs(k) < 0.02 ? 0 : k;
        return setpoint;
    }

    public double getDriveRotAmp() {
        double k = rotController.calculate(-targeting.rotToAmp(), 0);
        k += rotController.getSetpoint().velocity;
        double setpoint = Math.abs(targetRot) < 0.02 ? 0 : k;
        return setpoint;
    }

    public double getDriveRotOther90() {
        double k = rotController.calculate(-targeting.rotToOther90(), 0);
        k += rotController.getSetpoint().velocity;
        double setpoint = Math.abs(targetRot) < 0.02 ? 0 : k;
        return setpoint;
    }

    public double getDriveXCenter() {
        double k = 2 * (FieldConstants.kFieldLength / 2 - targeting.pose().getX());
        double setpoint = Math.abs(k) < 0.04 ? 0 : k;
        return setpoint;
    }

    public double getRot() {
        if (this.mode == DriveMode.SHOOT_AT_AMP) {
            double k = rotController.calculate(-targeting.rotToAmp(), 0);
            k += rotController.getSetpoint().velocity;
            double setpoint = Math.abs(targetRot) < 0.025 ? 0 : k;
            return setpoint;
        }
        double k =
                rotController.calculate(targetRot, 0)
                        * MathUtil.clamp(
                                Math.abs(swerve.getChassisSpeeds().vyMetersPerSecond / 1.5),
                                1,
                                100);
        // * MathUtil.clamp(
        //         3
        //                 / (Math.abs(swerve.getChassisSpeeds().omegaRadiansPerSecond)
        //                         + 0.1),
        //         1,
        //         4);
        k += rotController.getSetpoint().velocity;
        if (Math.abs(k) < 0.5 && swerve.getChassisSpeeds().vyMetersPerSecond < 0.5) {
            k = 0.5 * Math.signum(k);
        }
        Logger.recordOutput("k", k);

        double setpoint = Math.abs(targetRot) < 0.03 ? 0 : k;
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
