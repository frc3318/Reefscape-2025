package frc.robot;

import com.pathplanner.lib.events.EventTrigger;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
	private static RobotContainer m_robotContainer = new RobotContainer();
	public boolean extakeTrigger = false;

	/* Controllers */
	public final CommandXboxController driver = new CommandXboxController(0);

	/* Drive Controls */
	private final int translationAxis = XboxController.Axis.kLeftY.value;
	private final int strafeAxis = XboxController.Axis.kLeftX.value;
 	private final int rotationAxis = XboxController.Axis.kRightX.value ;

	/* Limiters */
  	private final SlewRateLimiter translationLimiter = new SlewRateLimiter(5.0);
  	private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(5.0);
  	private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(5.0);


  	/* Subsystems */
  	public final Swerve s_Swerve = new Swerve();
	public final WinchSubsystem s_Winch = new WinchSubsystem();
	public final ShooterSubsystem s_Shooter = new ShooterSubsystem();

  	private RobotContainer() 
  	{
    DogLog.setOptions(
        new DogLogOptions()
        .withCaptureConsole(true)
        .withCaptureDs(true)
        .withCaptureNt(true)
        .withLogEntryQueueCapacity(1000)
        .withLogExtras(true)
	);

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> translationLimiter.calculate(driver.getRawAxis(translationAxis)),
            () -> strafeLimiter.calculate(driver.getRawAxis(strafeAxis)),
            () -> rotationLimiter.calculate(driver.getRawAxis(rotationAxis)/2),
            () -> false
		)
	);

	configureButtonBindings();
  }

  	public static RobotContainer getInstance() {
    	return m_robotContainer;
  	}

	private void configureButtonBindings()
	{
		final Trigger lowShooter = driver.a();
		final Trigger highShooter = driver.x();
		final Trigger resetShooter = driver.y();
		final Trigger doHang = driver.leftBumper();
		final Trigger resetHang = driver.rightBumper();

		lowShooter.onTrue(s_Shooter.lowShoot());
		highShooter.onTrue(s_Shooter.highShooter());
		resetShooter.onTrue(s_Shooter.resetShooter());
		doHang.onTrue(s_Winch.doHang());
		resetHang.onTrue(s_Winch.resetHang());

		new EventTrigger("Extake").onTrue(new InstantCommand(() -> s_Shooter.lowShoot()));
		// this is because pathplanner is broken, don't ask me why we need two for it to actually shoot in Auton
		new EventTrigger("extake34").onTrue(new InstantCommand(() -> s_Shooter.lowShoot()));
	}
}