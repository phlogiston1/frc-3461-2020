/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallHandling extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public Spark hopper_l = new Spark(Constants.HOPPER_MOTOR_L);
  public Spark hopper_r = new Spark(Constants.HOPPER_MOTOR_R);
  public Spark chimney_l = new Spark(Constants.CHIMNEY_MOTOR_L);
  public Spark chimney_r = new Spark(Constants.CHIMNEY_MOTOR_R);
  //public DigitalInput beamBreak = new DigitalInput(); TODO beam break
  public BallHandling() {
  }
  public void setHopperSpeed(double speed){
    hopper_l.set(speed); //todo check directions
    hopper_r.set(-speed);
  }
  public void setHopperSpeed(double rSpeed, double lSpeed){
    hopper_r.set(rSpeed);
    hopper_l.set(lSpeed);
  }
  public void setChimneySpeed(double speed){
    chimney_l.set(speed); //todo check directions
    chimney_r.set(-speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
