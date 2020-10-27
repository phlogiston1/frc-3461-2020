/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * control the turret and hood positions
 */
public class Turret extends SubsystemBase {
  //yess baby neo on turret
  CANSparkMax turretMotor = new CANSparkMax(Constants.TURRET_PORT, MotorType.kBrushless);

  public boolean autoAiming = true; //todo remember what this is for

  public CANEncoder turretEnc = turretMotor.getEncoder();
  public CANPIDController pid = turretMotor.getPIDController(); //pid for turret

  /**
   * Creates a new Turret.
   */
  public Turret() {
    //set pid gains
    pid.setP(Constants.TURRET_POSITION_kP);
    pid.setI(Constants.TURRET_POSITION_kI);
    pid.setD(Constants.TURRET_POSITION_kD);
    pid.setFeedbackDevice(turretEnc);
  }
  /**
   * set the rotational velocity of the turret
   * @param speed speed from -1 to 1 pls
   */
  public void setSpeed(double speed){
    //todo add cap on speed here bcus speed limit doesn't apply to pid loop (which sends values < 1) but make sure thats okay because I haven't done testing in awhile
    speed *=  0.2; //todo add to constants after doing ^^

    //soft limit to make sure nothing breaks
    if((getPosition() > -180 || speed > 0) && (getPosition() < 180 || speed < 0)){
      turretMotor.set(speed);
    }else{
      turretMotor.set(0);
    }
  }

  //TODO BUILD THE ---- HOOD SO I CAN WRITE THIS
  public void setHoodPosition(double position){

  }

  /**
   * get the turret rotation
   * @return the turret position converted to degrees
   */
  public double getPosition(){
    return (turretEnc.getPosition() / 20 * 10 / 108) * 360;
  }

  /**
   * get turret position as rotation2d
   * @return rotation2d turret position
   */
  public Rotation2d getRotation(){
    return Rotation2d.fromDegrees(turretEnc.getPosition() * 360);
  }

  /**
   * move to rotation
   * @param rotation Rotation2d that the turret must move to
   */
  public void gotoRotation(Rotation2d rotation){
    pid.setReference(rotation.getDegrees()/360, ControlType.kPosition); //FIXME conversions for turret
  }

  //new thing replaces some commands
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //see if auto aim button is pressed. Testing only ??
    autoAiming = RobotContainer.getInstance().getOperatorJoystick().getRawButton(1);

    if(!autoAiming){
      //if manual aim, set speed to the joystick control
      setSpeed(RobotContainer.getInstance().getOperatorJoystick().getZ());
    }

    //button to reset turret encoder
    if(RobotContainer.getInstance().getOperatorJoystick().getRawButton(7)){
      turretEnc.setPosition(0); //todo testing only
    }
    SmartDashboard.putNumber("turret position", turretEnc.getPosition());
  }
}
