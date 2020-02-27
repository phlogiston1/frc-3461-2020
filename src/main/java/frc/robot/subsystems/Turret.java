/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;

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

public class Turret extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax turretMotor = new CANSparkMax(Constants.TURRET_PORT, MotorType.kBrushless);
  public boolean autoAiming = true;
  public CANEncoder turretEnc = turretMotor.getEncoder();
  public CANPIDController pid = turretMotor.getPIDController();
  public Turret() {
    pid.setP(Constants.TURRET_POSITION_kP);
    pid.setI(Constants.TURRET_POSITION_kI);
    pid.setD(Constants.TURRET_POSITION_kD);
    pid.setFeedbackDevice(turretEnc);
  }
  public void setSpeed(double speed){
    speed *=  0.2;
    if((getPosition() > -180 || speed > 0) && (getPosition() < 180 || speed < 0)){ //soft limit
      turretMotor.set(speed);
      System.out.println("setting turret speed to " + speed);
    }else{
      turretMotor.set(0);
    }
  }
  public void setHoodPosition(double position){

  }
  public double getPosition(){
    return (turretEnc.getPosition() / 20 * 10 / 108) * 360;
  }
  //public Rotation2d getRotation(){
    //return Roataion2d.fromDegreesturretEnc.getPosition();
  //}
  public Rotation2d getRotation(){
    return Rotation2d.fromDegrees(turretEnc.getPosition() * 360);
  }
  public void gotoRotation(Rotation2d rotation){
    pid.setReference(rotation.getDegrees()/360, ControlType.kPosition); //FIXME conversions for turret
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    autoAiming = RobotContainer.getInstance().getOperatorJoystick().getRawButton(1);
    if(!autoAiming){
      setSpeed(RobotContainer.getInstance().getOperatorJoystick().getZ());
    }
    if(RobotContainer.getInstance().getOperatorJoystick().getRawButton(5)){
      turretEnc.setPosition(0);
    }
    SmartDashboard.putNumber("turret position", turretEnc.getPosition());
  }
}
