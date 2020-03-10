/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2,3);
  public Intake() {
    setDefaultCommand(new IntakeOff(this));
  }
  public void setSpeed(double speed) {
    System.out.println(speed);
    intakeMotor.set(speed);
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
