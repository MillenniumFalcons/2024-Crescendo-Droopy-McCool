package team3647.frc2023.subsystems;

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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team3647.frc2023.util.VisionMeasurement;
import team3647.lib.PeriodicSubsystem;

public class SwerveDrive extends SwerveDrivetrain implements PeriodicSubsystem {

    private final SwerveDriveKinematics kinematics;

    public final Field2d field = new Field2d();

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final Pose2d zeroPose2d = new Pose2d();

    private final double kDt;

    private double pitchZero = 0;

    private double cachedSpeed = 0;

    public static class PeriodicIO {
        // inputs

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
        public FieldCentric fieldCentric =
                new FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        public RobotCentric robotCentric =
                new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    public SwerveDrive(
            SwerveDrivetrainConstants swerveDriveConstants,
            SwerveDriveKinematics kinematics,
            double maxSpeedMpS,
            double maxRotRadPerSec,
            double kDt,
            SwerveModuleConstants... swerveModuleConstants) {
        super(swerveDriveConstants, swerveModuleConstants);
        this.kinematics = kinematics;
        this.maxSpeedMpS = maxSpeedMpS;
        this.maxRotRadPerSec = maxRotRadPerSec;
        this.kDt = kDt;
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

        // SmartDashboard.putNumber("characterization voltage", periodicIO.characterizationVoltage);
        SmartDashboard.putData("field bruh", field);

        SmartDashboard.putNumber("x", getPoseX());
        SmartDashboard.putNumber("y", getPoseY());
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

    public void setCharacterizationVoltage(double voltage) {
        periodicIO.characterizationVoltage = voltage;
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
        return this.kinematics.toChassisSpeeds(getModuleStates());
    }

    public double getAccel() {
        double accel = (getChassisSpeeds().vxMetersPerSecond - cachedSpeed) / 0.02;
        this.cachedSpeed = getChassisSpeeds().vxMetersPerSecond;
        return accel;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getOdoRot());
    }

    public void addVisionData(VisionMeasurement data) {
        periodicIO.visionPose = data.pose;
        SmartDashboard.putNumber("timestamped viison", data.timestamp);
        this.m_odometry.addVisionMeasurement(data.pose, data.timestamp);
    }

    @Override
    public void end() {
        stopModules();
    }

    public void stopModules() {
        periodicIO.masterRequest = new SwerveRequest.Idle();
    }

    public void drive(double x, double y, double rotation) {
        periodicIO.robotCentric.withVelocityX(x).withVelocityY(y).withRotationalRate(rotation);
        periodicIO.masterRequest = periodicIO.robotCentric;
    }

    public void driveFieldOriented(double x, double y, double rotation) {
        periodicIO.fieldCentric.withVelocityX(x).withVelocityY(y).withRotationalRate(rotation);
        periodicIO.masterRequest = periodicIO.fieldCentric;
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            periodicIO.frontLeftState,
            periodicIO.frontRightState,
            periodicIO.backLeftState,
            periodicIO.backRightState
        };
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
