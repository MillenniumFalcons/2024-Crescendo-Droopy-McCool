package team3647.frc2024.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team3647.frc2024.constants.SwerveDriveConstants;
import team3647.frc2024.util.ModifiedSignalLogger;
import team3647.frc2024.util.SwerveFOCRequest;
import team3647.frc2024.util.VisionMeasurement;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.team254.swerve.SwerveKinematicLimits;
import team3647.lib.team254.swerve.SwerveSetpoint;
import team3647.lib.team254.swerve.SwerveSetpointGenerator;

public class SwerveDrive extends SwerveDrivetrain implements PeriodicSubsystem {

    public final SwerveSetpointGenerator setpointGenerator;

    public final SwerveKinematicLimits limits;

    public final Field2d field = new Field2d();

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final Pose2d zeroPose2d = new Pose2d();

    private final double kDt;

    private double pitchZero = 0;

    private double cachedSpeed = 0;

    private SysIdRoutine m_driveSysIdRoutine;

    private SysIdRoutine m_steerSysIdRoutine;

    public static class PeriodicIO {
        // inputs

        public SwerveSetpoint setpoint =
                new SwerveSetpoint(
                        new team3647.lib.team254.swerve.ChassisSpeeds(),
                        new team3647.lib.team254.swerve.SwerveModuleState
                                [SwerveDriveConstants.kDriveKinematics.getNumModules()]);

        public boolean good = false;

        public double characterizationVoltage = 0;
        public boolean isOpenloop = true;
        public double heading = 0;
        public double roll = 0;
        public double pitch = 0;
        public double rawHeading = 0;
        public Rotation2d gyroRotation = new Rotation2d();

        public SwerveModuleState frontLeftState = new SwerveModuleState();
        public SwerveModuleState frontRightState = new SwerveModuleState();
        public SwerveModuleState backLeftState = new SwerveModuleState();
        public SwerveModuleState backRightState = new SwerveModuleState();

        public double timestamp = 0;

        public Pose2d visionPose = new Pose2d();

        public SwerveRequest masterRequest = new SwerveRequest.Idle();
        public SwerveFOCRequest driveFOCRequest = new SwerveFOCRequest(true);
        public SwerveFOCRequest steerFOCRequest = new SwerveFOCRequest(false);
        public FieldCentric fieldCentric =
                new FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        public RobotCentric robotCentric =
                new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    public SwerveDrive(
            SwerveDrivetrainConstants swerveDriveConstants,
            double maxSpeedMpS,
            double maxRotRadPerSec,
            double kDt,
            SwerveModuleConstants... swerveModuleConstants) {
        super(swerveDriveConstants, swerveModuleConstants);
        this.maxSpeedMpS = maxSpeedMpS;
        this.maxRotRadPerSec = maxRotRadPerSec;
        this.kDt = kDt;

        this.setpointGenerator = new SwerveSetpointGenerator(SwerveDriveConstants.kDriveKinematics);

        this.limits = SwerveDriveConstants.kTeleopKinematicLimits;

        this.m_driveSysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.of(10).per(Units.Seconds.of(1)),
                                Units.Volts.of(30),
                                Units.Seconds.of(4),
                                ModifiedSignalLogger.logState()),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> volts) ->
                                        setControl(
                                                periodicIO.driveFOCRequest.withVoltage(
                                                        volts.in(Units.Volts))),
                                null,
                                this));

        this.m_steerSysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.of(3).per(Units.Seconds.of(1)),
                                Units.Volts.of(10),
                                Units.Seconds.of(4),
                                ModifiedSignalLogger.logState()),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> volts) ->
                                        setControl(
                                                periodicIO.steerFOCRequest.withVoltage(
                                                        volts.in(Units.Volts))),
                                null,
                                this));
    }

    public void zeroPitch() {
        this.pitchZero = this.getPitch();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.roll = this.m_pigeon2.getRoll().getValue();
        periodicIO.heading = this.m_pigeon2.getYaw().getValue();
        periodicIO.pitch = this.m_pigeon2.getPitch().getValue() - this.pitchZero;
        periodicIO.rawHeading = this.m_pigeon2.getYaw().getValue();
        periodicIO.frontLeftState = this.Modules[0].getCurrentState();
        periodicIO.frontRightState = this.Modules[1].getCurrentState();
        periodicIO.backLeftState = this.Modules[2].getCurrentState();
        periodicIO.backRightState = this.Modules[3].getCurrentState();
        periodicIO.gyroRotation = Rotation2d.fromDegrees(periodicIO.heading);
        periodicIO.timestamp = Timer.getFPGATimestamp();

        SmartDashboard.putBoolean("good", periodicIO.good);

        // SmartDashboard.putNumber("characterization voltage", periodicIO.characterizationVoltage);
    }

    @Override
    public void writePeriodicOutputs() {
        this.setControl(periodicIO.masterRequest);
        SmartDashboard.putNumber("heading", getRawHeading());

        // frontLeft.goForwardForCharacterization(periodicIO.characterizationVoltage);
        // frontRight.goForwardForCharacterization(periodicIO.characterizationVoltage);
        // backLeft.goForwardForCharacterization(periodicIO.characterizationVoltage);
        // backRight.goForwardForCharacterization(periodicIO.characterizationVoltage);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Robot/Output", this.m_odometry.getEstimatedPosition());
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public void reset() {
        for (int i = 0; i < 4; ++i) {
            periodicIO.setpoint.mModuleStates[i] =
                    new team3647.lib.team254.swerve.SwerveModuleState(
                            0.0,
                            team3647.lib.team254.geometry.Rotation2d.fromRadians(
                                    m_moduleStates[i].angle.getRadians()));
        }
        periodicIO.good = true;
    }

    public boolean underStage() {
        return ((getOdoPose().getX() > 3.2 && getOdoPose().getX() < 6.5)
                        || (getOdoPose().getX() > 9.9 && getOdoPose().getX() < 13.3))
                && ((Math.abs(getOdoPose().getY() - 4) < ((getOdoPose().getX() - 2.6) * 1 / 1.73)
                                && getOdoPose().getX() < 6.5)
                        || (Math.abs(getOdoPose().getY() - 4)
                                        < ((13.9 - getOdoPose().getX()) * 1 / 1.73)
                                && getOdoPose().getX() > 9.9));
    }

    public Command runDriveQuasiTest(Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction) {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    public void setRobotPose(Pose2d pose) {
        m_odometry.resetPosition(this.m_pigeon2.getRotation2d(), getModulePositions(), pose);
        periodicIO = new PeriodicIO();
    }

    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            this.Modules[i].getCANcoder().setPosition(0);
        }
    }

    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public void zeroGyro() {
        this.m_pigeon2.setYaw(0.0);
    }

    public double getHeading() {
        return periodicIO.heading;
    }

    public double getRoll() {
        return periodicIO.roll;
    }

    public double getPitch() {
        return periodicIO.pitch;
    }

    private void reduceCancoderStatusframes() {
        // this.backLeft.getCanCoderObject().setStatusFramePeriod(CANCoderStatusFrame.SensorData,
        // 255);
        // this.backRight
        // .getCanCoderObject()
        // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
        // this.frontLeft
        // .getCanCoderObject()
        // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
        // this.frontRight
        // .getCanCoderObject()
        // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    }

    // Probably want to moving average filter pitch and roll.
    public boolean isBalanced(double thresholdDeg) {
        return Math.abs(getRoll()) < thresholdDeg && Math.abs(getPitch()) < thresholdDeg;
    }

    public double getRawHeading() {
        return periodicIO.rawHeading;
    }

    @AutoLogOutput
    public Pose2d getOdoPose() {
        return this.m_odometry.getEstimatedPosition();
    }

    public Rotation2d getOdoRot() {
        return getOdoPose().getRotation();
    }

    public double getPoseX() {
        return this.m_odometry.getEstimatedPosition().getX();
    }

    public double getPoseY() {
        return this.m_odometry.getEstimatedPosition().getY();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.Modules[0].getPosition(true),
            this.Modules[1].getPosition(true),
            this.Modules[2].getPosition(true),
            this.Modules[3].getPosition(true)
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public double getAccel() {
        double accel = (getChassisSpeeds().vxMetersPerSecond - cachedSpeed) / 0.02;
        this.cachedSpeed = getChassisSpeeds().vxMetersPerSecond;
        return accel;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getOdoRot());
    }

    public boolean shouldAddData(Pose2d visionPose) {
        double distance = visionPose.minus(getOdoPose()).getTranslation().getNorm();
        double angle = visionPose.getRotation().minus(getOdoPose().getRotation()).getDegrees();
        return (distance < 1.5 && Math.abs(angle) < 15) ? true : false;
    }

    public void addVisionData(VisionMeasurement data) {
        periodicIO.visionPose = data.pose;
        SmartDashboard.putNumber("timestamped viison", data.timestamp);
        addVisionMeasurement(data.pose, data.timestamp, data.stdDevs);
    }

    @Override
    public void end() {
        stopModules();
    }

    public void stopModules() {
        periodicIO.masterRequest = new SwerveRequest.Idle();
    }

    public void drive(double x, double y, double rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint = generateRobotOriented(x, y, rotation);
        periodicIO
                .robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        periodicIO.masterRequest = periodicIO.robotCentric;
    }

    public SwerveSetpoint generate(double x, double y, double omega) {
        var robotRel =
                team3647.lib.team254.swerve.ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        omega,
                        team3647.lib.team254.geometry.Rotation2d.fromDegrees(
                                this.getOdoPose().getRotation().getDegrees()));
        periodicIO.setpoint =
                setpointGenerator.generateSetpoint(limits, periodicIO.setpoint, robotRel, kDt);
        return periodicIO.setpoint;
    }

    public SwerveSetpoint generateRobotOriented(double x, double y, double omega) {
        var robotRel = new team3647.lib.team254.swerve.ChassisSpeeds(x, y, omega);
        periodicIO.setpoint =
                setpointGenerator.generateSetpoint(limits, periodicIO.setpoint, robotRel, kDt);
        return periodicIO.setpoint;
    }

    public void driveFieldOriented(double x, double y, double rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint = generate(x, y, rotation);
        SmartDashboard.putNumber("generated x", setpoint.mChassisSpeeds.vxMetersPerSecond);
        periodicIO
                .robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        periodicIO.masterRequest = periodicIO.robotCentric;
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            periodicIO.frontLeftState,
            periodicIO.frontRightState,
            periodicIO.backLeftState,
            periodicIO.backRightState
        };
    }

    public double getVel() {
        return this.getChassisSpeeds().vxMetersPerSecond * this.getChassisSpeeds().vxMetersPerSecond
                + this.getChassisSpeeds().vyMetersPerSecond
                        * this.getChassisSpeeds().vyMetersPerSecond;
    }

    public double getMaxSpeedMpS() {
        return this.maxSpeedMpS;
    }

    public double getMaxRotationRadpS() {
        return this.maxRotRadPerSec;
    }

    @Override
    public String getName() {
        return "Swerve Drivetrain";
    }
}
