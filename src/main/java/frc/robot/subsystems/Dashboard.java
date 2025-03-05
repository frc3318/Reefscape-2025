package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.shuffleboard.*;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
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

    private static SwerveModuleState[] moduleStates = swerve.getModuleStates();

    private static boolean driveStatus;
    private static boolean angleStatus;

    private static GenericEntry[][] modulePositionEntries = new GenericEntry[4][2];
    private static GenericEntry[][] angleMotorEntries = new GenericEntry[4][8];
    private static GenericEntry[][] driveMotorEntries = new GenericEntry[4][8];


    public Dashboard()
    {
        SwerveModule[] modules = swerve.mSwerveMods;
        swerveOverview.add(odometryField);

        for (int i = 0; i < 4; i++)
        {
            driveStatus = (modules[i].mDriveMotor.isConnected());
            angleStatus = (modules[i].mAngleMotor.isConnected());

            /* Module Position Information */
            ShuffleboardLayout modulePosition = tabs[i].getLayout("Module Position", BuiltInLayouts.kGrid).withSize(3, 3);

            modulePositionEntries[i][0] = modulePosition.add("Gyro", moduleStates[i].angle.toString()).withWidget("Gyro").getEntry();
            modulePositionEntries[i][1] = modulePosition.add("Module Status", (angleStatus && driveStatus)).getEntry();

            /* Angle Motor Information */
            ShuffleboardLayout angleMotor = tabs[i].getLayout("Angle Motor", BuiltInLayouts.kGrid).withSize(4, 2);
            angleMotorEntries[i][0] = angleMotor.add("Angle Status", angleStatus).getEntry();
            angleMotorEntries[i][1] = angleMotor.add("Angle Temperature", modules[i].mAngleMotor.getDeviceTemp().toString()).getEntry();
            angleMotorEntries[i][2] = angleMotor.add("Angle Faults", modules[i].mAngleMotor.getFaultField().toString()).withWidget("Graph").getEntry();
            angleMotorEntries[i][3] = angleMotor.add("Angle Sticky Faults", modules[i].mAngleMotor.getStickyFaultField().toString()).withWidget("Graph").getEntry();
            angleMotorEntries[i][4] = angleMotor.add("Angle Voltage", modules[i].mAngleMotor.getMotorVoltage().toString()).getEntry();
            angleMotorEntries[i][5] = angleMotor.add("Angle Current", modules[i].mAngleMotor.getStatorCurrent().toString()).getEntry();
            angleMotorEntries[i][6] = angleMotor.add("Angle Velocity", modules[i].mAngleMotor.getRotorVelocity().toString()).getEntry();
            angleMotorEntries[i][7] = angleMotor.add("Angle Torque", modules[i].mAngleMotor.getMotorKT().toString()).getEntry();

            /* Drive Motor Information */
            ShuffleboardLayout driveMotor = tabs[i].getLayout("Drive Motor", BuiltInLayouts.kGrid).withSize(4, 2);
            driveMotorEntries[i][0] = driveMotor.add("Drive Status", driveStatus).getEntry();
            driveMotorEntries[i][1] = driveMotor.add("Drive Temperature", modules[i].mDriveMotor.getDeviceTemp().toString()).getEntry();
            driveMotorEntries[i][2] = driveMotor.add("Drive Faults", modules[i].mDriveMotor.getFaultField().toString()).withWidget("Graph").getEntry();
            driveMotorEntries[i][3] = driveMotor.add("Drive Sticky Faults", modules[i].mDriveMotor.getStickyFaultField().toString()).withWidget("Graph").getEntry();
            driveMotorEntries[i][4] = driveMotor.add("Drive Voltage", modules[i].mDriveMotor.getMotorVoltage().toString()).getEntry();
            driveMotorEntries[i][5] = driveMotor.add("Drive Current", modules[i].mDriveMotor.getStatorCurrent().toString()).getEntry();
            driveMotorEntries[i][6] = driveMotor.add("Drive Velocity", modules[i].mDriveMotor.getRotorVelocity().toString()).getEntry();
            driveMotorEntries[i][7] = driveMotor.add("Drive Torque", modules[i].mDriveMotor.getMotorKT().toString()).getEntry();
        }
    }

    public void update()
    {
        odometryField.setRobotPose(mRobotContainer.s_Swerve.getPose());

        SwerveModule[] modules = swerve.mSwerveMods;

        for (int i = 0; i < 4; i++)
        {
            driveStatus = (modules[i].mDriveMotor.isConnected());
            angleStatus = (modules[i].mAngleMotor.isConnected());
            /*
            for (GenericEntry modulePosition[] : modulePositionEntries) {
                modulePosition[0].setString(moduleStates[i].angle.toString());
                modulePosition[1].setBoolean(angleStatus && driveStatus);
            }
                 */

            /*
            for (GenericEntry angleMotor[] : angleMotorEntries) {
                angleMotor[0].setBoolean(angleStatus);
                angleMotor[1].setString(modules[i].mAngleMotor.getDeviceTemp().toString());
                angleMotor[2].setString(modules[i].mAngleMotor.getFaultField().toString());
                angleMotor[3].setString(modules[i].mAngleMotor.getStickyFaultField().toString());
                angleMotor[4].setString(modules[i].mAngleMotor.getMotorVoltage().toString());
                angleMotor[5].setString(modules[i].mAngleMotor.getStatorCurrent().toString());
                angleMotor[6].setString(modules[i].mAngleMotor.getRotorVelocity().toString());
                angleMotor[7].setString(modules[i].mAngleMotor.getMotorKT().toString());
            }

            for (GenericEntry driveMotor[] : driveMotorEntries) {
                driveMotor[0].setBoolean(driveStatus);
                driveMotor[1].setString(modules[i].mDriveMotor.getDeviceTemp().toString());
                driveMotor[2].setString(modules[i].mDriveMotor.getFaultField().toString());
                driveMotor[3].setString(modules[i].mDriveMotor.getStickyFaultField().toString());
                driveMotor[4].setString(modules[i].mDriveMotor.getMotorVoltage().toString());
                driveMotor[5].setString(modules[i].mDriveMotor.getStatorCurrent().toString());
                driveMotor[6].setString(modules[i].mDriveMotor.getRotorVelocity().toString());
                driveMotor[7].setString(modules[i].mDriveMotor.getMotorKT().toString());
            }
         */
        }
    }
}
