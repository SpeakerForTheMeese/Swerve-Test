package frc1512.TestRobot.lib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public ChassisSpeeds chassisSpeeds;
    public SwerveModuleState[] moduleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] states) {
        this.chassisSpeeds = chassisSpeeds;
        this.moduleStates = states;
    }
}
