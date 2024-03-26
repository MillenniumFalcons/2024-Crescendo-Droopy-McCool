package team3647.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public abstract class TalonFXSubsystem implements PeriodicSubsystem {

    private final TalonFX master;
    private final List<TalonFX> followers = new ArrayList<>();
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
    private final MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0);
    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC =
            new MotionMagicExpoTorqueCurrentFOC(0);
    private final MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
            new VelocityTorqueCurrentFOC(0);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);
    public ControlRequest controlMode = new EmptyControl();
    private Follower masterOutput;
    private final double positionConversion;
    private final double velocityConversion;
    private final double nominalVoltage;
    protected final double kDt;
    public static final int kLongStatusTimeMS = 255;
    public static final int kTimeoutMS = 100;
    private PeriodicIOAutoLogged ioAutoLogged = new PeriodicIOAutoLogged();

    protected TalonFXSubsystem(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        this.master = master;
        this.velocityConversion = velocityConversion;
        this.positionConversion = positionConversion;
        this.nominalVoltage = nominalVoltage;
        this.kDt = kDt;
        this.master.clearStickyFaults(kLongStatusTimeMS);
    }

    @AutoLog
    public static class PeriodicIO {
        // Inputs
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double timestamp = 0;
        public double masterCurrent = 0;
        public double nativePosition = 0;

        // Outputs
        public double demand = 0;
        public double feedforward = 0;
    }

    @Override
    public void readPeriodicInputs() {
        ioAutoLogged.nativePosition = master.getRotorPosition().getValue();
        ioAutoLogged.position = ioAutoLogged.nativePosition * positionConversion;
        ioAutoLogged.velocity = master.getRotorVelocity().getValue() * velocityConversion;
        ioAutoLogged.current = master.getStatorCurrent().getValue();
        ioAutoLogged.timestamp = Timer.getFPGATimestamp();
        ioAutoLogged.masterCurrent = master.getStatorCurrent().getValue();
        Logger.processInputs(getName(), ioAutoLogged);
    }

    @Override
    public void writePeriodicOutputs() {
        master.setControl(controlMode);
        master.setControl(controlMode);
        for (var follower : followers) {
            follower.setControl(masterOutput);
        }
    }

    @Override
    public void end() {
        setOpenloop(0);
    }

    public void setOpenloop(double output) {
        controlMode = dutyCycle;
        ioAutoLogged.feedforward = 0;
        dutyCycle.Output = output;
    }

    public void setVoltage(double voltage) {
        controlMode = voltageOut;
        ioAutoLogged.feedforward = 0;
        voltageOut.Output = voltage;
    }

    /**
     * Raw PID (not motion magic)
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPosition(double position, double feedforward) {
        controlMode = positionDutyCycle;
        positionDutyCycle.FeedForward = feedforward / nominalVoltage;
        positionDutyCycle.Position = position / positionConversion;
    }

    protected void setPositionNative(double position, double feedforward) {
        controlMode = positionDutyCycle;
        positionDutyCycle.FeedForward = feedforward / nominalVoltage;
        ioAutoLogged.feedforward = feedforward;
        positionDutyCycle.Position = position;
    }

    /**
     * Motion Magic position
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPositionMotionMagic(double position, double feedforward) {
        controlMode = motionMagicDutyCycle;
        motionMagicDutyCycle.Slot = 0;
        motionMagicDutyCycle.FeedForward = feedforward / nominalVoltage;
        motionMagicDutyCycle.Position = position / positionConversion;
    }

    /**
     * Voltage position
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPositionExpoVoltage(double position, double feedforward) {
        controlMode = motionMagicExpoVoltage;
        motionMagicExpoVoltage.Slot = 0;
        motionMagicExpoVoltage.FeedForward = feedforward / nominalVoltage;
        motionMagicExpoVoltage.Position = position / positionConversion;
    }

    protected void setPositionExpoVoltage(double position, double feedforward, int slot) {
        controlMode = motionMagicExpoVoltage;
        motionMagicExpoVoltage.Slot = slot;
        motionMagicExpoVoltage.FeedForward = feedforward / nominalVoltage;
        motionMagicExpoVoltage.Position = position / positionConversion;
    }

    /**
     * Voltage position
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param velocity in units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setPositionVoltage(double position, double velocity, double feedforward) {
        controlMode = positionVoltage;
        positionVoltage.Slot = 0;
        positionVoltage.Velocity = velocity / velocityConversion;
        positionVoltage.FeedForward = feedforward / nominalVoltage;
        positionVoltage.Position = position / positionConversion;
    }

    /**
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPositionFOC(double position, double feedforward) {
        controlMode = motionMagicExpoTorqueCurrentFOC;
        motionMagicExpoTorqueCurrentFOC.FeedForward = feedforward;
        motionMagicExpoTorqueCurrentFOC.Position = position / positionConversion;
        motionMagicExpoTorqueCurrentFOC.Slot = 0;
    }

    /**
     * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setVelocityFOC(double velocity, double feedforward) {
        controlMode = velocityTorqueCurrentFOC;
        velocityTorqueCurrentFOC.Slot = 0;
        velocityTorqueCurrentFOC.Acceleration = velocity / velocityConversion;
        velocityTorqueCurrentFOC.FeedForward = feedforward;
        velocityTorqueCurrentFOC.Velocity = velocity / velocityConversion;
    }

    /**
     * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setVelocity(double velocity, double feedforward) {
        controlMode = velocityDutyCycle;
        velocityDutyCycle.Acceleration = velocity / 2.0 / velocityConversion;
        velocityDutyCycle.FeedForward = feedforward / nominalVoltage;
        velocityDutyCycle.Velocity = velocity / velocityConversion;
    }

    /**
     * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setVelocityVoltage(double velocity, double feedforward) {
        controlMode = velocityVoltage;
        velocityVoltage.Slot = 0;
        velocityDutyCycle.Acceleration = velocity / 2.0 / velocityConversion;
        velocityVoltage.Velocity = velocity / velocityConversion;
        velocityVoltage.FeedForward = feedforward / nominalVoltage;
    }

    /**
     * @param torque
     */
    protected void setTorque(double torque) {
        controlMode = torqueCurrentFOC;
        torqueCurrentFOC.Output = torque;
    }

    /** Sets all motors to brake mode */
    public void setToBrake() {
        setNeutralMode(NeutralModeValue.Brake);
    }

    /** Sets all motors to coast mode */
    public void setToCoast() {
        setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets master and all followers to the mode
     *
     * @param mode either Brake or Coast
     */
    public void setNeutralMode(NeutralModeValue mode) {
        TalonFXConfigurator masterConfigurator = master.getConfigurator();
        TalonFXConfiguration masterConfiguration = new TalonFXConfiguration();
        masterConfigurator.refresh(masterConfiguration);
        masterConfiguration.MotorOutput.NeutralMode = mode;
        masterConfigurator.apply(masterConfiguration);
    }

    /** Sets the selected sensor to 0 (default) */
    public void resetEncoder() {
        setEncoder(0);
    }

    /** Sets the selected sensor to 0 (default) */
    public void resetEncoder(int timeoutMS) {
        master.setPosition(0, timeoutMS);
    }

    /**
     * sets the selected sensor to position
     *
     * @param position position in output units
     */
    protected void setEncoder(double position) {
        master.setPosition(position / positionConversion);
    }
    /**
     * @return the velocity in the output units
     */
    public double getVelocity() {
        return ioAutoLogged.velocity;
    }

    /**
     * @return ths position in the output units
     */
    public double getPosition() {
        return ioAutoLogged.position;
    }

    /**
     * @return the timestamp for the position and velocity measurements
     */
    public double getTimestamp() {
        return ioAutoLogged.timestamp;
    }

    public double getMasterCurrent() {
        return ioAutoLogged.masterCurrent;
    }

    public double getNativePos() {
        return ioAutoLogged.nativePosition;
    }

    protected void addFollower(TalonFX follower, boolean opposeMaster) {
        this.masterOutput = new Follower(this.master.getDeviceID(), opposeMaster);
        followers.add(follower);
    }

    protected void setStatusFrames(TalonFX device, int statusLengthMS, int timeoutMS) {
        // device.setStatusFramePeriod(StatusFrame.Status_1_General, statusLengthMS, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, statusLengthMS, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, statusLengthMS, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_6_Misc, statusLengthMS, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, statusLengthMS, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, statusLengthMS,
        // timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, statusLengthMS, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, statusLengthMS, timeoutMS);
        // device.setStatusFramePeriod(
        //         StatusFrame.Status_15_FirmwareApiStatus, statusLengthMS, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_17_Targets1, statusLengthMS, timeoutMS);
    }

    protected void setStatusFramesThatDontMatter(TalonFX device, int timeout, int timeoutMS) {
        // device.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, timeout, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_6_Misc, timeout, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, timeout, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, timeout, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, timeout, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, timeout, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, timeout, timeoutMS);
        // device.setStatusFramePeriod(StatusFrame.Status_17_Targets1, timeout, timeoutMS);
    }
}
