package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Dashboard {
    private static RobotContainer mRobotContainer = RobotContainer.getInstance();
    private static Swerve swerve = mRobotContainer.s_Swerve;;
    private static Field2d odometryField = new Field2d();

    private static ShuffleboardTab frontLeft = Shuffleboard.getTab("Front Left");
    private static ShuffleboardTab frontRight = Shuffleboard.getTab("Front Right");
    private static ShuffleboardTab rearLeft = Shuffleboard.getTab("Rear Left");
    private static ShuffleboardTab rearRight = Shuffleboard.getTab("Rear Right");

    private static ShuffleboardTab[] tabs = {frontLeft, frontRight, rearLeft, rearRight};
    private static ShuffleboardTab swerveOverview = Shuffleboard.getTab("Swerve Overview");


    public static void update()
    {
        odometryField.setRobotPose(mRobotContainer.s_Swerve.getPose());
        // swerveOverview.add(odometryField);

        SwerveModuleState[] moduleStates = swerve.getModuleStates();
        SwerveModule[] modules = swerve.mSwerveMods;

        

        boolean driveStatus;
        boolean angleStatus;

        for (int i = 0; i < 4; i++)
        {
            /* Module Position Information */
            driveStatus = (modules[i].mDriveMotor.isConnected());
            angleStatus = (modules[i].mAngleMotor.isConnected());

            tabs[i].add("Heading", moduleStates[i].angle.getMeasure()).withWidget("Gyro");
            tabs[i].add("Module Status", (angleStatus && driveStatus));

            /* Angle Motor Information */
            tabs[i].add("Angle Status", angleStatus);
            tabs[i].add("Angle Temperature", modules[i].mAngleMotor.getDeviceTemp().toString());
            tabs[i].add("Angle Faults", modules[i].mAngleMotor.getFaultField().toString()).withWidget("Graph");
            tabs[i].add("Angle Sticky Faults", modules[i].mAngleMotor.getStickyFaultField().toString()).withWidget("Graph");
            tabs[i].add("Angle Voltage", modules[i].mAngleMotor.getMotorVoltage().toString());
            tabs[i].add("Angle Current", modules[i].mAngleMotor.getStatorCurrent().toString());
            tabs[i].add("Angle Velocity", modules[i].mAngleMotor.getRotorVelocity().toString());
            tabs[i].add("Angle Torque", modules[i].mAngleMotor.getMotorKT().toString());

            /* Drive Motor Information */
            tabs[i].add("Drive Status", driveStatus);
            tabs[i].add("Drive Temperature", modules[i].mDriveMotor.getDeviceTemp().toString());
            tabs[i].add("Drive Faults", modules[i].mDriveMotor.getFaultField().toString()).withWidget("Graph");
            tabs[i].add("Drive Sticky Faults", modules[i].mDriveMotor.getStickyFaultField().toString()).withWidget("Graph");
            tabs[i].add("Drive Voltage", modules[i].mDriveMotor.getMotorVoltage().toString());
            tabs[i].add("Drive Current", modules[i].mDriveMotor.getStatorCurrent().toString());
            tabs[i].add("Drive Velocity", modules[i].mDriveMotor.getRotorVelocity().toString());
            tabs[i].add("Drive Torque", modules[i].mDriveMotor.getMotorKT().toString());
        }
    }
}
