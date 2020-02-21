/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
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
  CANEncoder encoder = turretMotor.getEncoder();
  public boolean autoAiming = false;
  public Turret() {
  }
  public void setSpeed(double speed){
    speed *=  0.2;
    if((getPosition() > 0 || speed > 0) && (getPosition() < 35 || speed < 0)){ //soft limit
      turretMotor.set(speed);
    }else{
      turretMotor.set(0);
    }
  }
  public double getPosition(){
    return encoder.getPosition();
  }
  public Rotation2d getRotation(){
    return Rotation2d.fromDegrees(encoder.getPosition());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    autoAiming = RobotContainer.getInstance().getOperatorJoystick().getRawButton(1);
    if(!autoAiming){
      setSpeed(RobotContainer.getInstance().getOperatorJoystick().getX());
    }
    if(RobotContainer.getInstance().getOperatorJoystick().getRawButton(5)){
      encoder.setPosition(-10);
    }
    SmartDashboard.putNumber("turret position", encoder.getPosition());
  }
}
