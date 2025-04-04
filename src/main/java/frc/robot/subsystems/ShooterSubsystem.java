package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public static final SparkMax motor = new SparkMax(9, MotorType.kBrushless);

    private static boolean isMotorMoving;
    
    public Command lowShoot()
    {
        return Commands.startEnd(
            () -> ShooterSubsystem.set(-0.7), 
            () -> ShooterSubsystem.set(0), 
            this
        );
    }

    public Command highShooter()
    {
        return Commands.startEnd(
            () -> ShooterSubsystem.set(-1), 
            () -> ShooterSubsystem.set(0), 
            this
        );
    }

    public Command resetShooter()
    {
        return Commands.startEnd(
            () -> ShooterSubsystem.set(1), 
            () -> ShooterSubsystem.set(0), 
            this
        );
    }

    public static void set(double speed)
    {
        motor.set(speed);

        isMotorMoving = (speed != 0);
    }

    public boolean getShooterMoving()
    {
        return isMotorMoving;
    }

    public double getShooterPosition()
    {
        return motor.getAbsoluteEncoder().getPosition();
    }
}