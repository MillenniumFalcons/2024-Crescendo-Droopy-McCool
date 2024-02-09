package team3647.frc2024.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class SwerveFOCRequest implements SwerveRequest {
    private final MotionMagicVoltage m_motionMagicControl =
            new MotionMagicVoltage(0, false, 0, 0, false, false, false);
    private final TorqueCurrentFOC m_torqueCurrentFOC = new TorqueCurrentFOC(0.0);

    private double m_targetTorque = 0.0;
    private boolean m_driveType = true;

    public SwerveFOCRequest(boolean driveType) {
        m_driveType = driveType;
    }

    public SwerveFOCRequest() {
        m_driveType = true;
    }

    @Override
    public StatusCode apply(
            SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for (var module : modulesToApply) {
            if (m_driveType) {
                // Command steer motor to zero
                module.getSteerMotor().setControl(m_motionMagicControl);

                // Command drive motor to voltage
                module.getDriveMotor().setControl(m_torqueCurrentFOC.withOutput(m_targetTorque));
            } else {
                // Command steer motor to voltage
                module.getSteerMotor().setControl(m_torqueCurrentFOC.withOutput(m_targetTorque));

                // Command drive motor to zero
                module.getDriveMotor().setControl(m_motionMagicControl);
            }
        }

        return StatusCode.OK;
    }

    /**
     * @param targetTorque Torque for all modules to target
     * @return
     */
    public SwerveFOCRequest withVoltage(double targetTorque) {
        this.m_targetTorque = targetTorque;
        return this;
    }
}
