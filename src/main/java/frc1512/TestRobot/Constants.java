package frc1512.TestRobot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc1512.TestRobot.Subsystems.SwerveModule;
import frc1512.TestRobot.lib.SwerveSetpointGenerator;
import frc1512.TestRobot.lib.SwerveSetpointGenerator.KinematicLimits;

public class Constants {
    public static final double EPSILON = 1e-9;
    public static final double UPDATE_PERIOD = 0.02;
    public static final class ModuleConstants {
        public static final double TurningkP = 0.5;
        public static final double kWheelDiameterMeters = 0;
        public static final double kDrivingMotorGearRatio = 0;
        public static final double kTurningMotorGearRatio = 0;
        public static final double kDriveEncoderRot2Meter = kDrivingMotorGearRatio * Math.PI * kWheelDiameterMeters ;
        public static final double kDriveEncoderRot2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * Math.PI *2;
        public static final double kTurningEncoderRot2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final SwerveModule.ModuleConfiguration NORTH_WEST_CONFIG = 
            new SwerveModule.ModuleConfiguration();
        static {
            //setup configs here pls
        }
        public static final SwerveModule.ModuleConfiguration NORTH_EAST_CONFIG = 
            new SwerveModule.ModuleConfiguration();
        static {
            //setup configs here pls
        }
        public static final SwerveModule.ModuleConfiguration SOUTH_WEST_CONFIG = 
            new SwerveModule.ModuleConfiguration();
        static {
            //setup configs here pls
        }
        public static final SwerveModule.ModuleConfiguration SOUTH_EAST_CONFIG = 
            new SwerveModule.ModuleConfiguration();
        static {
            //setup configs here pls
        }

    }
    public static final class DriveConstants {
        public static final double kPhysicalMaxSpeedMetersPerSec = 0.0;
        public static final double kMaxTranslationMPS = 4.0;
        public static final double kMaxRotationSpeedRad = 1.0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double MAINTAIN_kP = 4.0;
        public static final double MAINTAIN_kI = 0.0;
        public static final double MAINTAIN_kD = 0.1;

        public static final double PROFILE_CONSTRAINT_VEL = 0.0;
        public static final double PROFILE_CONSTRAINT_ACCEL = 0.0;

        public static final double kTrackWidth = Units.inchesToMeters(0);
        public static final double kWheelBase = Units.inchesToMeters(0);
        public static final SwerveDriveKinematics KDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2)
        );
        public static final KinematicLimits KINEMATIC_LIMITS = new KinematicLimits();

        static {
            KINEMATIC_LIMITS.maxDriveVelocity = 5.3; // m/s
            KINEMATIC_LIMITS.maxDriveAcceleration = 25; // m/s^2
            KINEMATIC_LIMITS.maxSteeringVelocity = 25; // rad/s
        }

    }
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDeadband = 0.01;
      }
      
}
