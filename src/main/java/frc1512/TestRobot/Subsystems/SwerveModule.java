package frc1512.TestRobot.Subsystems;




import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc1512.TestRobot.Constants;

public class SwerveModule extends SubsystemBase{
   
private final CANSparkMax _driveMotor;
private final CANSparkMax _turningMotor;

private final RelativeEncoder _driveEncoder;
private final RelativeEncoder _turningEncoder;

private final PIDController _angleController;

private final AnalogInput _absEncoder;
private final boolean _abosoluteEncoderReversed;
private final double _absOffsetRad;

private final Translation2d _positionVector;

public SwerveModule(
     SwerveModule.ModuleConfiguration config,
     int drivemotorid,
     int turningmotorid,
     int absencoderid)
   {

    this._absOffsetRad = config.absencoderoffset;
    this._abosoluteEncoderReversed = config.abosoluteEncoderReversed;
    _absEncoder = new AnalogInput(absencoderid);

    _driveMotor = new CANSparkMax(drivemotorid, MotorType.kBrushless);
    _turningMotor = new CANSparkMax(turningmotorid, MotorType.kBrushless);

    _driveMotor.setInverted(config.drivemotorreversed);
    _turningMotor.setInverted(config.turningmotorreversed);

    _driveEncoder = _driveMotor.getEncoder();
    _turningEncoder = _turningMotor.getEncoder();

    _driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
    _driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2MeterPerSec);
    _turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderRot2Rad);
    _turningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRot2RadPerSec);

    
    _angleController = new PIDController(Constants.ModuleConstants.TurningkP,0,0);
    _angleController.enableContinuousInput(-Math.PI, Math.PI);
    _positionVector = config.position;
    
    

   // resetEncoders();

   }
   public double getDrivePosition(){
        return _driveEncoder.getPosition();
   }
   public double getTurningPosition(){
        return _turningEncoder.getPosition();
   }
   public double getDriveVelocity(){
        return _driveEncoder.getVelocity();
   }
   public double getTurningVelocity(){
            return _turningEncoder.getVelocity();
   }
   public SwerveModulePosition getModluePosition(){
     return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(Units.radiansToDegrees(getabsEncoderRad())));
   }
   public Rotation2d getModlueAngle() {
          return new Rotation2d(Units.radiansToDegrees(getabsEncoderRad()));
   }
   public double getabsEncoderRad(){
        double angle = _absEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= _absOffsetRad;
        return angle * (_abosoluteEncoderReversed ? -1.0 : 1.0);
   }
   public void resetEncoders(){
     _driveEncoder.setPosition(0.0);
     _turningEncoder.setPosition(getabsEncoderRad());
   }
    // location is x, y position w.r.t robot frame
    public Translation2d getModuleLocation() {
     return _positionVector;
 }

 public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
 }
 public void setDesiredState(SwerveModuleState state){
     if (Math.abs(state.speedMetersPerSecond) < 0.001){
          stopModule();
          return;
     }

    state = SwerveModuleState.optimize(state, getState().angle);
    _driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSec);
    _turningMotor.set(_angleController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve" + _absEncoder.getChannel() + "state", state.toString());
 }
public void stopModule(){
     _driveMotor.set(0);
     _turningMotor.set(0);
}
public static class ModuleConfiguration {
     public String moduleName = "";

     public Translation2d position = new Translation2d();

     public double encoderOffset = 0.0;
     public boolean encoderInverted = false;

     public String canBus = "rio";
     public boolean drivemotorreversed = false;
     public boolean turningmotorreversed = false; 
     public double absencoderoffset = 0.0; 
     public boolean abosoluteEncoderReversed = false; 
 }
}





