package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private static RobotContainer m_robotContainer = new RobotContainer();
	public boolean extakeTrigger = false;

	/* Controllers */
	private final CommandXboxController driver = new CommandXboxController(0);

	/* Drive Controls */
	private final int translationAxis = XboxController.Axis.kLeftY.value;
	private final int strafeAxis = XboxController.Axis.kLeftX.value;
 	private final int rotationAxis = XboxController.Axis.kRightX.value ;

	/* Limiters */
  	private final SlewRateLimiter translationLimiter = new SlewRateLimiter(5.0);
  	private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(5.0);
  	private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0);

  	public JoystickButton lowShooter = new JoystickButton(driver.getHID(), XboxController.Button.kA.value);

  	public JoystickButton highShooter = new JoystickButton(driver.getHID(), XboxController.Button.kX.value);

  	public JoystickButton zeroGyro = new JoystickButton(driver.getHID(), XboxController.Button.kStart.value);

  	public JoystickButton intakeReset = new JoystickButton(driver.getHID(), XboxController.Button.kY.value);

  	/* Motor Controller */
  	public final SparkMax ExtakeMotor = new SparkMax(9, MotorType.kBrushless);

  	/* Subsystems */
  	public final Swerve s_Swerve = new Swerve();

  	/* Auton Selection */
  	private final SendableChooser<Command> autoChooser;

  	private RobotContainer() 
  	{
    	DogLog.setOptions(
            new DogLogOptions()
            .withCaptureConsole(true)
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withLogEntryQueueCapacity(1000)
            .withLogExtras(true));
            
    extakeTrigger = (lowShooter.getAsBoolean() || highShooter.getAsBoolean() || intakeReset.getAsBoolean());
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    new EventTrigger("Extake").whileTrue(new InstantCommand(() -> ExtakeMotor.set(-0.8)));
    new EventTrigger("Extake").whileFalse(new InstantCommand(() -> ExtakeMotor.set(0)));

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> translationLimiter.calculate(driver.getRawAxis(translationAxis)),
            () -> strafeLimiter.calculate(driver.getRawAxis(strafeAxis)),
            () -> rotationLimiter.calculate(driver.getRawAxis(rotationAxis)/2),
            () -> false));


    lowShooter.onTrue(new InstantCommand(() -> ExtakeMotor.set(-.6)));
    lowShooter.onFalse(new InstantCommand(() -> ExtakeMotor.set(0)));

    highShooter.onTrue(new InstantCommand(() -> ExtakeMotor.set(-1)));
    highShooter.onFalse(new InstantCommand(() -> ExtakeMotor.set(0)));

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    intakeReset.onTrue(new InstantCommand(() -> ExtakeMotor.set(2)));
    intakeReset.onFalse(new InstantCommand(() -> ExtakeMotor.set(0)));
  }

  	public static RobotContainer getInstance() {
    	return m_robotContainer;
  	}



  	public Command getAutonomousCommand() {
    	return autoChooser.getSelected();
  	}
}