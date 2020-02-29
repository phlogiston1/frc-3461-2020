/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.IntakeOff;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private CANSparkMax intake = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2,3);
  public Intake() {
    setDefaultCommand(new IntakeOff(this));
  }
  public void setSpeed(double speed) {
    intake.set(speed);
  }
  public void extend(){
    intakeSolenoid.set(Value.kForward);
  }
  public void retract(){
    intakeSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
