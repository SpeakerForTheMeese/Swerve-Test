package frc1512.TestRobot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc1512.TestRobot.Constants;
import frc1512.TestRobot.OI;
import frc1512.TestRobot.Subsystems.DriveTrain;


public class Drive extends CommandBase {
    private final DriveTrain _drivetrain;
    private final OI _oi;
    private final SlewRateLimiter _xLimiter, _yLimiter, _rotLimiter;

    public Drive(DriveTrain driveTrain, OI oi){
this._drivetrain = driveTrain;
this._oi = oi;
this._xLimiter = new SlewRateLimiter(Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSec);
this._yLimiter = new SlewRateLimiter(Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSec);
this._rotLimiter  = new SlewRateLimiter(Constants.DriveConstants.kMaxRotationSpeedRad);
addRequirements(driveTrain);
    }
    @Override
    public void initialize() {
        
        
    }
    @Override
    public void execute() {
       double Xspeed  = _oi.getX();
       double Yspeed = _oi.GetY();
       double RotSpeed = _oi.getRot();
       
       Xspeed = Math.abs(Xspeed) > Constants.OperatorConstants.kDeadband ? Xspeed : 0.0;
       Yspeed = Math.abs(Yspeed) > Constants.OperatorConstants.kDeadband ? Yspeed : 0.0;
       RotSpeed = Math.abs(RotSpeed) > Constants.OperatorConstants.kDeadband ? RotSpeed : 0.0; 

       Xspeed = _xLimiter.calculate(Xspeed) * Constants.DriveConstants.kMaxTranslationMPS;
       Yspeed = _yLimiter.calculate(Yspeed) * Constants.DriveConstants.kMaxTranslationMPS;
       RotSpeed = _rotLimiter.calculate(RotSpeed) * Constants.DriveConstants.kMaxRotationSpeedRad;

       ChassisSpeeds chassisSpeeds;
       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        Xspeed, Yspeed, RotSpeed, _drivetrain.getHeadingRotation2d());

        SwerveModuleState[] moduleStates = Constants.DriveConstants.
        KDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        _drivetrain.setModuleStates(moduleStates);
       

    }
    @Override
    public void end(boolean interrupted) {
        _drivetrain.stopAll();
        
    }
    @Override
    public boolean isFinished() {
        return false;
        
    }

    
    








    
}
