package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;

public class OI {
  protected  Joystick _driverGamepad;
  protected Joystick _operatorGamepad;
  protected Joystick _rotoStick;
  public OI(){

  }
  public double GetY(){
    return _driverGamepad.getY();
  }
  public double getX(){
    return _driverGamepad.getX();
  }
  public double getRot(){
    return _rotoStick.getX();
  }
}

