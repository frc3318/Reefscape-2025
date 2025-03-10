package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.shuffleboard.*;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;

import frc.robot.RobotContainer;
import frc.robot.SwerveModule;

public class Dashboard {
    private static RobotContainer mRobotContainer = RobotContainer.getInstance();
    private static Swerve swerve = mRobotContainer.s_Swerve;;
    private static Field2d odometryField = new Field2d();

    /* Create dashboard tabs for each Swerve module */
    private static ShuffleboardTab frontLeft = Shuffleboard.getTab("Front Left");
    private static ShuffleboardTab frontRight = Shuffleboard.getTab("Front Right");
    private static ShuffleboardTab rearLeft = Shuffleboard.getTab("Rear Left");
    private static ShuffleboardTab rearRight = Shuffleboard.getTab("Rear Right");

    /* Put these tabs into an array to be looped and populated */
    private static ShuffleboardTab[] tabs = {frontLeft, frontRight, rearLeft, rearRight};
    private static ShuffleboardTab swerveOverview = Shuffleboard.getTab("Swerve Overview");

    /* Array for Module States */
    private static SwerveModuleState[] moduleStates = swerve.getModuleStates();

    /* Temporary variables to store status for each module part */
    private static boolean driveStatus;
    private static boolean angleStatus;
    private static boolean encoderStatus;

    /* Stores the value to be updated for each tab, first index is which module to access (0-3), second index is the index of the information type to access */
    /* For these two, [0] for the second index stores the heading of the module, and [1] stores the velocity of the module */
    private static GenericEntry[][] OverviewModulePositionEntries = new GenericEntry[4][2];
    private static GenericEntry[][] modulePositionEntries = new GenericEntry[4][2];

    /* List of all indexes and their information stored
     * [0] = boolean Status
     * [1] = double Motor Temperature
     * [2] = double Fault Count
     * [3] = double Sticky Fault Count
     * [4] = double Voltage
     * [5] = double Current
     * [6] = double Velocity
     * [7] = double Torque
     */
    private static GenericEntry[][] angleMotorEntries = new GenericEntry[4][8];
    private static GenericEntry[][] driveMotorEntries = new GenericEntry[4][8];


    public Dashboard()
    {
        SwerveModule[] modules = swerve.mSwerveMods;
        SwerveData swerveData = new SwerveData();

        /* Initialize swerve module visualization data */
        swerveData.setModuleLocations(swerve.getModulePositions());
        swerveData.setModuleSpeeds(swerve.getSpeeds().omegaRadiansPerSecond);
        swerveData.setHeading(swerve.getHeading());
        swerveData.setXVelocity(swerve.getSpeeds().vxMetersPerSecond);
        swerveData.setYVelocity(swerve.getSpeeds().vyMetersPerSecond);

        /* Adds field element to overview tab */
        swerveOverview.add(odometryField).withSize(20, 6).withPosition(9, 0);
        swerveOverview.add("Swerve", swerveData).withWidget("SwerveDrive").withPosition(16, 5);

        ShuffleboardLayout modulePositions = swerveOverview.getLayout("Module Headings", BuiltInLayouts.kGrid).withSize(9, 11).withProperties(Map.of("Number of Columns", 2, "Number of Rows", 2));
        ShuffleboardLayout moduleVelocities = swerveOverview.getLayout("Module Velocities", BuiltInLayouts.kGrid).withSize(7, 5).withProperties(Map.of("Number of Columns", 2, "Number of Rows", 2)).withPosition(9, 6);
        
        for (int i = 0; i < 4; i++)
        {
            driveStatus = (modules[i].mDriveMotor.isConnected());
            angleStatus = (modules[i].mAngleMotor.isConnected());
            encoderStatus = (modules[i].getCANcoder() != null);

            /* Module Heading Information */
            ShuffleboardLayout modulePosition = tabs[i].getLayout("Module Information", BuiltInLayouts.kGrid).withSize(8, 11).withPosition(0, 0).withProperties(Map.of("Number of Columns", 1, "Number of Rows", 2));

            OverviewModulePositionEntries[i][0] = modulePositions.add(String.format("Module %s Heading", i + 1), moduleStates[i].angle.getDegrees()).withWidget("Gyro").withPosition((i % 2), (i < 2 ? 0 : 1)).getEntry();
            OverviewModulePositionEntries[i][1] = moduleVelocities.add(String.format("Module %s Velocity", i + 1), moduleStates[i].speedMetersPerSecond).withWidget("Number Bar").withPosition((i % 2), (i < 2 ? 0 : 1)).getEntry();

            modulePositionEntries[i][0] = modulePosition.add("Heading", moduleStates[i].angle.getDegrees()).withWidget("Gyro").withPosition(0, 0).getEntry();
            modulePositionEntries[i][1] = modulePosition.add("Module Status", (angleStatus && driveStatus && encoderStatus)).withPosition(0, 1).getEntry();

            /* Angle Motor Information */
            ShuffleboardLayout angleMotor = tabs[i].getLayout("Angle Motor", BuiltInLayouts.kGrid).withSize(21, 5).withPosition(8, 0).withProperties(Map.of("Number of Columns", 4, "Number of Rows", 2));

            angleMotorEntries[i][0] = angleMotor.add("Status", angleStatus)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();
            angleMotorEntries[i][1] = angleMotor.add("Temperature", modules[i].mAngleMotor.getDeviceTemp().getValueAsDouble())
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();
            angleMotorEntries[i][2] = angleMotor.add("Faults", modules[i].mAngleMotor.getFaultField().getValueAsDouble())
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();
            angleMotorEntries[i][3] = angleMotor.add("Sticky Faults", modules[i].mAngleMotor.getStickyFaultField().getValueAsDouble())
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();
            angleMotorEntries[i][4] = angleMotor.add("Voltage", modules[i].mAngleMotor.getMotorVoltage().getValueAsDouble())
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();
            angleMotorEntries[i][5] = angleMotor.add("Current", modules[i].mAngleMotor.getStatorCurrent().getValueAsDouble())
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();
            angleMotorEntries[i][6] = angleMotor.add("Velocity", modules[i].mAngleMotor.getRotorVelocity().getValueAsDouble())
                .withPosition(3, 0)
                .withSize(1, 1)
                .getEntry();
            angleMotorEntries[i][7] = angleMotor.add("Torque", modules[i].mAngleMotor.getMotorKT().getValueAsDouble())
                .withPosition(3, 1)
                .withSize(1, 1)
                .getEntry();

            /* Drive Motor Information */
            ShuffleboardLayout driveMotor = tabs[i].getLayout("Drive Motor", BuiltInLayouts.kGrid).withSize(21, 5).withPosition(8, 6).withProperties(Map.of("Number of Columns", 4, "Number of Rows", 2));

            driveMotorEntries[i][0] = driveMotor.add("Status", driveStatus)
                .withPosition(0, 1)
                .getEntry();
            driveMotorEntries[i][1] = driveMotor.add("Temperature", modules[i].mDriveMotor.getDeviceTemp().getValueAsDouble())
                .withPosition(0, 0)
                .getEntry();
            driveMotorEntries[i][2] = driveMotor.add("Faults", modules[i].mDriveMotor.getFaultField().getValueAsDouble())
                .withPosition(1, 0)
                .getEntry();
            driveMotorEntries[i][3] = driveMotor.add("Sticky Faults", modules[i].mDriveMotor.getStickyFaultField().getValueAsDouble())
                .withPosition(1, 1)
                .getEntry();
            driveMotorEntries[i][4] = driveMotor.add("Voltage", modules[i].mDriveMotor.getMotorVoltage().getValueAsDouble())
                .withPosition(2, 0)
                .getEntry();
            driveMotorEntries[i][5] = driveMotor.add("Current", modules[i].mDriveMotor.getStatorCurrent().getValueAsDouble())
                .withPosition(2, 1)
                .getEntry();
            driveMotorEntries[i][6] = driveMotor.add("Velocity", modules[i].mDriveMotor.getRotorVelocity().getValueAsDouble())
                .withPosition(3, 0)
                .getEntry();
            driveMotorEntries[i][7] = driveMotor.add("Torque", modules[i].mDriveMotor.getMotorKT().getValueAsDouble())
                .withPosition(3, 1)
                .getEntry();
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
            encoderStatus = (modules[i].getCANcoder() != null);
            

            OverviewModulePositionEntries[i][0].setDouble(moduleStates[i].angle.getDegrees());
            OverviewModulePositionEntries[i][1].setDouble(moduleStates[i].speedMetersPerSecond);
            
            for (GenericEntry modulePosition[] : modulePositionEntries) {
                modulePosition[0].setDouble(moduleStates[i].angle.getDegrees());
                modulePosition[1].setBoolean(angleStatus && driveStatus && encoderStatus);
            }

            /* Update all angle motor entry information */
            for (GenericEntry angleMotor[] : angleMotorEntries) {
                angleMotor[0].setBoolean(angleStatus);
                angleMotor[1].setDouble(modules[i].mAngleMotor.getDeviceTemp().getValueAsDouble());
                angleMotor[2].setDouble(modules[i].mAngleMotor.getFaultField().getValueAsDouble());
                angleMotor[3].setDouble(modules[i].mAngleMotor.getStickyFaultField().getValueAsDouble());
                angleMotor[4].setDouble(modules[i].mAngleMotor.getMotorVoltage().getValueAsDouble());
                angleMotor[5].setDouble(modules[i].mAngleMotor.getStatorCurrent().getValueAsDouble());
                angleMotor[6].setDouble(modules[i].mAngleMotor.getRotorVelocity().getValueAsDouble());
                angleMotor[7].setDouble(modules[i].mAngleMotor.getMotorKT().getValueAsDouble());
            }

            /* Update all drive motor entry information */
            for (GenericEntry driveMotor[] : driveMotorEntries) {
                driveMotor[0].setBoolean(driveStatus);
                driveMotor[1].setDouble(modules[i].mDriveMotor.getDeviceTemp().getValueAsDouble());
                driveMotor[2].setDouble(modules[i].mDriveMotor.getFaultField().getValueAsDouble());
                driveMotor[3].setDouble(modules[i].mDriveMotor.getStickyFaultField().getValueAsDouble());
                driveMotor[4].setDouble(modules[i].mDriveMotor.getMotorVoltage().getValueAsDouble());
                driveMotor[5].setDouble(modules[i].mDriveMotor.getStatorCurrent().getValueAsDouble());
                driveMotor[6].setDouble(modules[i].mDriveMotor.getRotorVelocity().getValueAsDouble());
                driveMotor[7].setDouble(modules[i].mDriveMotor.getMotorKT().getValueAsDouble());
            }
        }
    }
}
