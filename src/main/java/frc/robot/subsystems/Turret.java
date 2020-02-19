/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax turretMotor = new CANSparkMax(Constants.TURRET_PORT, MotorType.kBrushless);
  Encoder encoder = new Encoder(0,1,false,Encoder.EncodingType.k2X); //TODO
  public boolean autoAiming = true;
  public Turret() {
    encoder.setDistancePerPulse(4./256.); //TODO
  }
  public void setSpeed(double speed){
    turretMotor.set(speed);
  }
  public void setHoodPosition(double position){

  }
  public double getPosition(){
    return encoder.getDistance();
  }
  public Rotation2d getRotation(){
    return Rotation2d.fromDegrees(encoder.getDistance());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    autoAiming = RobotContainer.getInstance().getOperatorJoystick().getRawButton(1);
    if(!autoAiming){
      setSpeed(RobotContainer.getInstance().getOperatorJoystick().getX());
    }
  }
}
