package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDSubsystem;


public class Robot extends TimedRobot {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    private Command m_autonomousCommand;
    private Dashboard m_Dashboard;

    @Override
    public void robotInit() {
        // Start Subsystems
        m_Dashboard = new Dashboard();
        new LEDSubsystem();

        var alliance = DriverStation.getAlliance();

        DogLog.log("Misc/Robot Status", "Robot has Started");
        DogLog.log("DriverStation/Status", "Alliance recorded as " + alliance.toString());
    }

    /* Called every robot packet in every mode */
    @Override
    public void robotPeriodic() {
        // update SmartDashboard
        // dashboard.update();

        // update Command Scheduling (DO NOT REMOVE)
        // WILL break all robot mechanisms, required to listen for commands
        CommandScheduler.getInstance().run();
        DogLog.log("Misc/FMS Match Time", DriverStation.getMatchTime());
    }


    @Override
    public void disabledInit() {
        DogLog.log("Misc/Robot Status", "Robot has been disabled");
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        DogLog.log("Misc/Robot Status", "Auto has begun");

        m_autonomousCommand = m_Dashboard.autoChooser.getSelected();
        
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            DogLog.log("Misc/Robot Status", "Running auto command " + m_autonomousCommand.getName());
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        DogLog.log("Misc/Robot Status", "TeleOp has begun");
        // ensures Auton stops during Teleop
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
