package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchSubsystem extends SubsystemBase {
    public static final SparkMax motor = new SparkMax(2, MotorType.kBrushless);
    
    public Command doHang()
    {
        return Commands.startEnd(
            () -> WinchSubsystem.set(-0.7), 
            () -> WinchSubsystem.set(0), 
            this
        );
    }

    public Command resetHang()
    {
        return Commands.startEnd(
            () -> WinchSubsystem.set(0.7), 
            () -> WinchSubsystem.set(0), 
            this
        );
    }

    public static void set(double speed)
    {
        motor.set(speed);
    }
}
