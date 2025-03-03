package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.Swerve;

public class ReefscapeAuto extends SequentialCommandGroup {
    public ReefscapeAuto(Swerve s_Swerve, SparkMax neoMotor) {

        addCommands(
            // Drive forward using Translation2d
            //-3.657
             new InstantCommand( () -> s_Swerve.drive(new Translation2d(0, 0), 180, false, false)),
            //1.34 for 180
            //0.335 for 45
              new WaitCommand(1.34), // Run motor for 1 second
              new InstantCommand( () -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false)),
              new WaitCommand(0.5),

            new StartEndCommand(
                () -> s_Swerve.drive(new Translation2d(4, 0), 0, true, false), // Move forward (robot relative)
                () -> s_Swerve.drive(new Translation2d(0, 0), 0, true, false), // Stop moving
                s_Swerve
            ).withTimeout(4.0), // Ensure it runs for 2 seconds
            // Activate the motor mapped to A button
              new InstantCommand(() -> neoMotor.set(-0.6)), // Set motor speed
              new WaitCommand(1.0), // Run motor for 1 second 
             new InstantCommand(() -> neoMotor.set(0)) // Stop motor
        );
    }
}