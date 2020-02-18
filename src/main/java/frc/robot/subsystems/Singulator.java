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

public class Singulator extends SubsystemBase { //TODO add beam-break sensors
  /**
   * Creates a new ExampleSubsystem.
   */
  Spark lMotor = new Spark(Constants.SINGULATOR_MOTOR_L);
  Spark rMotor = new Spark(Constants.SINGULATOR_MOTOR_R);
  public Singulator() {

  }
  public void runSingulator(double speed){
    lMotor.set(speed);
    rMotor.set(speed);
  }
  public void runSingulator(double lSpeed, double rSpeed){
    lMotor.set(lSpeed);
    rMotor.set(rSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
