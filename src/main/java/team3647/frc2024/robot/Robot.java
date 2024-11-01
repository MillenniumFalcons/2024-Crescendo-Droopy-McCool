// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2024.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import team3647.lib.team6328.VirtualSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    public static final double kTenMSLoopTime = 0.01;
    public static final double kTwentyMSLoopTime = 0.02;

    private RobotContainer robotContainer = new RobotContainer();

    public Robot() {
        super(.02);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(
                    new WPILOGWriter("/home/lvuser/logs/")); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(true); // Run as fast as possible
            String logPath =
                    LogFileUtil
                            .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt
            // the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new NT4Publisher());
            // Logger.addDataReceiver(
            //         new WPILOGWriter(
            //                 LogFileUtil.addPathSuffix(
            //                         logPath, "_sim"))); // Save outputs to a new log
        }

        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the
        // "Understanding Data Flow" page
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata
        // values may be added.

        AutoLogOutputManager.addPackage("frc.lib");

        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);
        SignalLogger.enableAutoLogging(false);
        // SignalLogger.setPath("/home/lvuser/logs/");
        SignalLogger.stop();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // // autonomous chooser on the dashboard.
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        VirtualSubsystem.periodicAll();
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {

        robotContainer.allianceChecker.periodic();
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        robotContainer.swerve.zeroPitch();
        autonomousCommand = robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.configTestCommands();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
