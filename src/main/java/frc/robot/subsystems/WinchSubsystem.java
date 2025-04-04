package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class WinchSubsystem extends SubsystemBase {
    private SparkMax winch;
    
    

    public WinchSubsystem()
    {
        // winch = RobotContainer.getInstance().winch;
    }

    @Override
    public void periodic() {
        
    }
}
