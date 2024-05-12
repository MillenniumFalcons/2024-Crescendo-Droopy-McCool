package team3647.frc2024.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
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

        public double cachedVel = 0;
        public boolean isAccel = false;

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

        public Pose2d pose = new Pose2d();
        public ChassisSpeeds speeds = new ChassisSpeeds();

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
        registerTelemetry(this::setStuff);
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

        AutoBuilder.configureHolonomic(
                this::getOdoPose, // Robot pose supplier
                this::setRobotPose, // Method to reset odometry (will be called if your auto has
                // a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (ChassisSpeeds speeds) ->
                        this.drive(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond,
                                speeds.omegaRadiansPerSecond), // Method that will drive the robot
                // given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
                        // live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        5, // Max module speed, in m/s
                        0.35, // Drive base radius in meters. Distance from robot center to furthest
                        // module.
                        new ReplanningConfig() // Default path replanning config. See the API for
                        // the options here
                        ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
                );
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

        // SmartDashboard.putBoolean("good", periodicIO.good);

        // SmartDashboard.putNumber("characterization voltage", periodicIO.characterizationVoltage);
    }

    @Override
    public void writePeriodicOutputs() {
        setisAccel();
        this.setControl(periodicIO.masterRequest);
        // SmartDashboard.putNumber("heading", getRawHeading());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("is accel", getIsAccel());
        // Logger.recordOutput("Robot/Output", this.getOdoPose());
        Logger.recordOutput(
                "Drive/Encoder", this.Modules[0].getDriveMotor().getPosition().getValueAsDouble());
        Logger.recordOutput(
                "Drive/Current",
                this.Modules[0].getDriveMotor().getStatorCurrent().getValueAsDouble());
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public void setisAccel() {
        if (getVel() > periodicIO.cachedVel + 0.02) {
            periodicIO.isAccel = true;
        } else {
            periodicIO.isAccel = false;
        }
        periodicIO.cachedVel = getVel();
    }

    public boolean getIsAccel() {
        return periodicIO.isAccel;
    }

    public Command setAccelLimit(double limit) {
        return Commands.runOnce(() -> limits.kMaxDriveAcceleration = limit);
    }

    public void reset() {
        for (int i = 0; i < 4; ++i) {
            periodicIO.setpoint.mModuleStates[i] =
                    new team3647.lib.team254.swerve.SwerveModuleState(
                            0.0,
                            team3647.lib.team254.geometry.Rotation2d.fromRadians(
                                    this.m_modulePositions[i].angle.getRadians()));
        }
        periodicIO.good = true;
    }

    public boolean underStage() {
        return ((getOdoPose().getX() > 3.2 && getOdoPose().getX() < 6.5)
                        || (getOdoPose().getX() > 9.9 && getOdoPose().getX() < 13.3))

                && ((Math.abs(getOdoPose().getY() - 4)/*dist from mid */ < ((getOdoPose().getX() - 2.6) * 1 / 1.73 /*slope of stage*/)
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
        seedFieldRelative(pose);
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
        return periodicIO.pose;
    }

    public void setStuff(SwerveDriveState state) {
        // SignalLogger.writeDoubleArray(
        //         "odometry",
        //         new double[] {
        //             state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees()
        //         });
        periodicIO.pose = state.Pose;
        periodicIO.speeds = this.m_kinematics.toChassisSpeeds(state.ModuleStates);
        // SignalLogger.writeDoubleArray(
        //         "speeds",
        //         new double[] {
        //             periodicIO.speeds.vxMetersPerSecond,
        //             periodicIO.speeds.vyMetersPerSecond,
        //             periodicIO.speeds.omegaRadiansPerSecond
        //         });
    }

    public Rotation2d getOdoRot() {
        return getOdoPose().getRotation();
    }

    public double getPoseX() {
        return this.getOdoPose().getX();
    }

    public double getPoseY() {
        return this.getOdoPose().getY();
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
        return periodicIO.speeds;
    }

    public double getAccel() {
        double accel = (getChassisSpeeds().vxMetersPerSecond - cachedSpeed) / 0.02;
        this.cachedSpeed = getChassisSpeeds().vxMetersPerSecond;
        return accel;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getOdoRot());
    }

    public boolean shouldAddData(VisionMeasurement measurement) {
        double distance = measurement.pose.minus(getOdoPose()).getTranslation().getNorm();
        double angle =
                measurement.pose.getRotation().minus(getOdoPose().getRotation()).getDegrees();
        return (distance < MathUtil.clamp(getVel(), 0.25, 1.5) && Math.abs(angle) < 15)
                        || DriverStation.isAutonomous()
                ? true
                : false;
    }

    public void addVisionData(VisionMeasurement data) {
        periodicIO.visionPose = data.pose;
        // SmartDashboard.putNumber("timestamped viison", data.timestamp);
        // SignalLogger.writeDoubleArray(
        //         "vision pose",
        //         new double[] {
        //             data.pose.getX(),
        //             data.pose.getY(),
        //             data.pose.getRotation().getDegrees(),
        //             data.timestamp
        //         });
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
        periodicIO
                .robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        periodicIO.masterRequest = periodicIO.robotCentric;
    }

    public void driveFieldOriented(DoubleSupplier x, double y, double rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint = generate(x.getAsDouble(), y, rotation);
        periodicIO
                .robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        periodicIO.masterRequest = periodicIO.robotCentric;
    }

    public void driveFieldOriented(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint =
                generate(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble());
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

    public double getVel() { // squared
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
