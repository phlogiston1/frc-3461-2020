/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.commands.IntakeOff;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
public class Intake extends SubsystemBase {
  /**
   * Simple code to control intake
   */
  //self explanatory
  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_MOTOR);
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(INTAKE_SOLENOID_A,INTAKE_SOLENOID_B);

  public Intake() {
    //dont start spinning as soon as robot turned on.
    setDefaultCommand(new IntakeOff(this));
  }

  /**
   * set intake roller velocity
   * @param speed speed from -1 to 1
   */
  public void setSpeed(double speed) {
    System.out.println(speed);
    intakeMotor.set(speed);
  }

  /**
   * lower the intake
   */
  public void extend(){
    intakeSolenoid.set(Value.kForward);
  }

  /**
   * raise the intake
   */
  public void retract(){
    intakeSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
