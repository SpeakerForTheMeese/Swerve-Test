package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;


import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.lib.SwerveSetpoint;
import frc.robot.lib.SwerveSetpointGenerator;
import frc.robot.lib.SwerveSetpointGenerator.KinematicLimits;

public class DriveTrain  extends SubsystemBase{
    private final SwerveModule[] _modules;
    private static final int NORTH_EAST_MODULE_IDX = 1;
    private static final int NORTH_WEST_MODULE_IDX = 2;
    private static final int SOUTH_EAST_MODULE_IDX = 3;
    private static final int SOUTH_WEST_MODULE_IDX = 4;

    private ControlState _controlState;
    //private final BaseStatusSignalValue[] _moduleSignals;
    private ChassisIO _IO;

    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;
    private final SwerveDrivePoseEstimator _poseEstimator;
    

    private final SwerveSetpointGenerator _SwerveSetpointGenerator;
    private KinematicLimits _kinematicLimits = Constants.DriveConstants.KINEMATIC_LIMITS;

    private final HolonomicDriveController _poseController;

    private final Field2d _field;

    private final Pigeon2 _imu = new Pigeon2(0);
    private double _yawOffset = 0;
    
    public DriveTrain(
        RobotContainer container,
        Pigeon2 imu) {
        imu = _imu;
        
        int NUM_MODULES = 4;
        _controlState = ControlState.NEUTRAL;

        _modules = new SwerveModule[4];
        _modules[NORTH_EAST_MODULE_IDX] = new SwerveModule(Constants.ModuleConstants.NORTH_EAST_CONFIG, RobotMap.CAN.NE_DRIVE_ID, RobotMap.CAN.NE_ROT_ID, RobotMap.DIO.NE_ENCODER_ID);
        _modules[NORTH_WEST_MODULE_IDX] = new SwerveModule(Constants.ModuleConstants.NORTH_WEST_CONFIG, RobotMap.CAN.NW_DRIVE_ID, RobotMap.CAN.NW_ROT_ID, RobotMap.DIO.NW_ENCODER_ID);
        _modules[SOUTH_EAST_MODULE_IDX] = new SwerveModule(Constants.ModuleConstants.SOUTH_EAST_CONFIG, RobotMap.CAN.SE_DRIVE_ID, RobotMap.CAN.SE_ROT_ID, RobotMap.DIO.SE_ENCODER_ID);
        _modules[SOUTH_WEST_MODULE_IDX] = new SwerveModule(Constants.ModuleConstants.SOUTH_WEST_CONFIG, RobotMap.CAN.SW_DRIVE_ID, RobotMap.CAN.SW_ROT_ID, RobotMap.DIO.SW_ENCODER_ID);


        _controlState = ControlState.MANUEL;

        _kinematics = new SwerveDriveKinematics(
            _modules[NORTH_EAST_MODULE_IDX].getModuleLocation(),
            _modules[NORTH_WEST_MODULE_IDX].getModuleLocation(),
            _modules[SOUTH_EAST_MODULE_IDX].getModuleLocation(),
            _modules[SOUTH_EAST_MODULE_IDX].getModuleLocation());
        _odometry = new SwerveDriveOdometry(
            _kinematics,
            getRotation2d(),
            new SwerveModulePosition[] {
                _modules[NORTH_EAST_MODULE_IDX].getModluePosition(),
                _modules[NORTH_WEST_MODULE_IDX].getModluePosition(),
                _modules[SOUTH_EAST_MODULE_IDX].getModluePosition(),
                _modules[SOUTH_EAST_MODULE_IDX].getModluePosition()
            },
            new Pose2d(0, 0, getRotation2d()));
        _poseEstimator = new SwerveDrivePoseEstimator(
            _kinematics,
            getRotation2d(),
            new SwerveModulePosition[] {
                _modules[NORTH_EAST_MODULE_IDX].getModluePosition(),
                _modules[NORTH_WEST_MODULE_IDX].getModluePosition(),
                _modules[SOUTH_EAST_MODULE_IDX].getModluePosition(),
                _modules[SOUTH_WEST_MODULE_IDX].getModluePosition()
            },
            new Pose2d(0,0, getRotation2d()));
        
        _poseController = new HolonomicDriveController(
             new PIDController(
                Constants.DriveConstants.kP,
                Constants.DriveConstants.kI, 
                Constants.DriveConstants.kD), 
             new PIDController(
                Constants.DriveConstants.kP,
                Constants.DriveConstants.kI,
                Constants.DriveConstants.kD),
             new ProfiledPIDController(
                Constants.DriveConstants.MAINTAIN_kP,
                Constants.DriveConstants.MAINTAIN_kI,
                Constants.DriveConstants.MAINTAIN_kD,
                new TrapezoidProfile.Constraints(
                    Constants.DriveConstants.PROFILE_CONSTRAINT_VEL,
                    Constants.DriveConstants.PROFILE_CONSTRAINT_ACCEL))); 
        _SwerveSetpointGenerator = 
            new SwerveSetpointGenerator(
                _kinematics, 
                new Translation2d[] {
                    _modules[NORTH_EAST_MODULE_IDX].getModuleLocation(),
                    _modules[NORTH_WEST_MODULE_IDX].getModuleLocation(),
                    _modules[SOUTH_EAST_MODULE_IDX].getModuleLocation(),
                    _modules[SOUTH_WEST_MODULE_IDX].getModuleLocation()
                });

        _field = new Field2d();    
             
        // zeroGyroscope();
        // resetRobotPose(new Pose2d());  
    }
    public static class ChassisIO {
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measuredStates = 
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        SwerveModulePosition[] measuredPositions = 
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Rotation2d heading  = new Rotation2d(0.0);
        double pitch  = 0.0;
        Pose2d esitmatedPose = new Pose2d();
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4]);

    }
    @Override
    public void periodic() {
        super.periodic();
    
        readModules();
        updateDesiredStates();
        setModuleStates(_IO.setpoint.moduleStates);
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }
    

    public double getHeading(){
        return (Math.IEEEremainder(_imu.getYaw(), 360));
    }
 
    
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }
    public void plotTrajectory(Trajectory t, String name){
        _field.getObject(name).setTrajectory(t);
    }
    public void readModules() {
        for (int module = 0; module < _modules.length; module++) {
            _IO.measuredStates[module] = _modules[module].getState();
            _IO.measuredPositions[module]  = _modules[module].getModluePosition();
        }
    }
    public void orientModules(Rotation2d moduleAngle) {
        for (int module = 0; module < _modules.length; module++) {
            _IO.setpoint.moduleStates[module] = new SwerveModuleState(0.0, moduleAngle);
        }
    } 
        
    
    public void stopAll(){
        _modules[NORTH_EAST_MODULE_IDX].stopModule();
        _modules[NORTH_WEST_MODULE_IDX].stopModule();
        _modules[SOUTH_EAST_MODULE_IDX].stopModule();
        _modules[SOUTH_WEST_MODULE_IDX].stopModule();

    }
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, null, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSec, Constants.DriveConstants.kMaxTranslationMPS, Constants.DriveConstants.kMaxRotationSpeedRad );

        _modules[NORTH_EAST_MODULE_IDX].setDesiredState(desiredStates[0]);
        _modules[NORTH_WEST_MODULE_IDX].setDesiredState(desiredStates[1]);
        _modules[SOUTH_EAST_MODULE_IDX].setDesiredState(desiredStates[2]);
        _modules[SOUTH_WEST_MODULE_IDX].setDesiredState(desiredStates[3]);
    }
    public void setControlState(ControlState state) {
        _controlState = state;
    }
    public ControlState getControlState() {
        return _controlState;
    }
    public SwerveSetpoint getSetpoint() {
        return _IO.setpoint;
    }
    public void setKinematicLimits(KinematicLimits limits) {
        if (limits !=  _kinematicLimits) {
            _kinematicLimits = limits;
        }
    }
    public void updateDesiredStates() {
        Pose2d robotPoseVel = 
            new Pose2d(
                _IO.desiredChassisSpeeds.vxMetersPerSecond * Constants.UPDATE_PERIOD,
                _IO.desiredChassisSpeeds.vyMetersPerSecond * Constants.UPDATE_PERIOD,
                    Rotation2d.fromRadians(
                        _IO.desiredChassisSpeeds.omegaRadiansPerSecond * Constants.UPDATE_PERIOD));
        Twist2d twistVel = new Pose2d().log(robotPoseVel);
        ChassisSpeeds updatedChassisSpeeds =
            new ChassisSpeeds(
                twistVel.dx / Constants.UPDATE_PERIOD,
                twistVel.dy / Constants.UPDATE_PERIOD,
                twistVel.dtheta / Constants.UPDATE_PERIOD);
        _IO.setpoint = 
                _SwerveSetpointGenerator.generateSetpoint(
                        _kinematicLimits,
                        _IO.setpoint,
                        updatedChassisSpeeds,
                        Constants.UPDATE_PERIOD);
        };
    public void setVelocity(ChassisSpeeds speeds) {
        _IO.desiredChassisSpeeds = speeds;
    }
    public void setVelocityPose(Pose2d pose)   {
        ChassisSpeeds speeds = 
            _poseController.calculate(
                _IO.esitmatedPose, pose, 0.0, _IO.heading); 
                
                       
    }              
            
                           
    
    public enum ControlState {
        NEUTRAL(0),
        MANUEL(1),
        TRAJECTORY(2);

        private final int _value;

        ControlState(int value) {
            _value = value;
        }
        public int getValue() {
            return _value;
        }
    }




}
