package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = (translationSup.getAsDouble() > Constants.stickDeadband ? 0.0 : deadband(translationSup.getAsDouble()));
        double strafeVal = (strafeSup.getAsDouble() > Constants.stickDeadband ? 0.0 : deadband(strafeSup.getAsDouble()));
        double rotationVal = (rotationSup.getAsDouble() > Constants.stickDeadband ? 0.0 : deadband(rotationSup.getAsDouble()));
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }

    // calculates optimized deadband
    private double deadband(double sup)
    {
            return (1 / (1 - Constants.stickDeadband) * (sup + (-sign(sup) * Constants.stickDeadband)));
    }

    // returns sign of value, 0 if 0
    private int sign(double sup)
    {
        int sign = 0;

        if (sup > 0)
        {
            sign = 1;
        } else if (sup < 0)
        {
            sign = -1;
        }

        return sign;
    }    
}