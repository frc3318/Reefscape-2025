package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        // public static final int pigeonID = 8;
        public static final String canBusID = "rio";


        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.5);
        public static final double wheelBase = Units.inchesToMeters(22.5); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.05;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = .19844;
        public static final double driveKV = 2.2097;
        public static final double driveKA = .46947;

        /* Swerve Profiling Values */
        /** Meters per Second */
        //was 4.78
        public static final double maxSpeed = 4.78;
        /** Radians per Second */
        public static final double maxAngularVelocity = 5;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 10;
            // Bezels left
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-78.22);
            // Bezels right
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-21.16-25.21);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 7;
            // Bezels left
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-174.46); 
            // Bezels right
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(26.78-16.60);
             public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 13;
            // Bezels left
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(34.98);
            // Bezels right
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-22.47-21.02+0.641);
            public static final SwerveModuleConstants constants = 
new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 3;
            // Bezels left
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(31.90);
            // Bezels right
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(25.82-(1.37+2.86));
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double kMass = 38.5;
        public static final double kMOI = 4.367;

        /* Auton Angle Motor PID Values */
        public static final double angleKP = 2.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 5;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.05;
        public static final double driveKF = 0.0;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class MaxRPMConstants {
        public static final double maxRPMNeo550 = 11000;
        public static final double maxRPMNeo = 5676;
        public static final double maxRPMVortex = 6780;
    }

    public static final class LEDs {
         /* LED arrangement */
        public static final int startIdx = 0;
        public static final int numLEDs = 43;
        public static final int totalLEDs = startIdx + numLEDs;
        /* Animations */
        private static final Distance kLedSpacing = Meters.of(1 / 61.51);
        public static final LEDPattern GSMSTGradient = LEDPattern.gradient(GradientType.kContinuous, Color.kWhite, Color.kDarkBlue);
        public static final LEDPattern ScrollingGradient = GSMSTGradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
        /* Misc */
        public static final double blinkRate = 0.2; // Regular blink rate
        public static final double errorBlinkRate = 0.1; // Blink rate for errors and warnings
        public static final double tempStateTime = 0.70; // How long for warnings and errors
    }
}
