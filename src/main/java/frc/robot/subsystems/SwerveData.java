package frc.robot.subsystems;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveData implements Sendable {
    // Instance variables for each property
    private double[] moduleLocations;
    private double[] moduleSpeeds;
    private double heading;
    private double xVelocity;
    private double yVelocity;
    private double maxVelocity;
    private double angularVelocity;

    // Constructor to initialize with some values
    public SwerveData() {
        // Initialize with default values (you should set these dynamically based on your robot's actual data)
        this.moduleLocations = new double[8];  // For example, 4 modules, each having (x, y)
        this.moduleSpeeds = new double[8];     // For example, 4 modules, each having (v, θ)
        this.heading = 0.0;
        this.xVelocity = 0.0;
        this.yVelocity = 0.0;
        this.maxVelocity = 5.0;
        this.angularVelocity = 0.0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // Set the type to SWERVE_DRIVE in Shuffleboard
        builder.setSmartDashboardType("SWERVE_DRIVE");

        // Add the properties to be displayed in Shuffleboard
        builder.addDoubleArrayProperty("Module Locations", 
            () -> moduleLocations,     // Supplier for moduleLocations array
            (newValues) -> moduleLocations = newValues); // Consumer to update moduleLocations array

        builder.addDoubleArrayProperty("Module Speeds", 
            () -> moduleSpeeds,     // Supplier for moduleSpeeds array
            (newValues) -> moduleSpeeds = newValues); // Consumer to update moduleSpeeds array

        builder.addDoubleProperty("Heading", () -> heading, (newHeading) -> heading = newHeading);
        builder.addDoubleProperty("X Velocity", () -> xVelocity, (newXVel) -> xVelocity = newXVel);
        builder.addDoubleProperty("Y Velocity", () -> yVelocity, (newYVel) -> yVelocity = newYVel);
        builder.addDoubleProperty("Max Velocity", () -> maxVelocity, (newMaxVel) -> maxVelocity = newMaxVel);
        builder.addDoubleProperty("Angular Velocity", () -> angularVelocity, (newAngularVel) -> angularVelocity = newAngularVel);
    }
    

    // Setters and Getters for the properties
    public void setModuleLocations(SwerveModulePosition[] moduleLocations) {
        for (int i = 0; i < 4; i++)
        {
            this.moduleLocations[i] = moduleLocations[i].distanceMeters;
        }
    }

    public void setModuleSpeeds(double moduleSpeeds) {
        for (int i = 0; i < 4; i++)
        {
            this.moduleSpeeds[i] = moduleSpeeds;
        }
    }

    public void setHeading(Rotation2d heading) {
        this.heading = heading.getDegrees();
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }
}
