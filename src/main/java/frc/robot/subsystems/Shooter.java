/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Shooter extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    private TalonFX masterTalon = new TalonFX(Constants.SHOOTER_MOTOR_1);
    private TalonFX slaveTalon = new TalonFX(Constants.SHOOTER_MOTOR_2);
    public Shooter() {
      slaveTalon.follow(masterTalon);
      masterTalon.setInverted(false); //TODO make sure motor is spinning forwards
      slaveTalon.setInverted(InvertType.FollowMaster); //TODO make sure motors aren't fighting
      masterTalon.config_kP(0, Constants.shooter_kP);
      masterTalon.config_kI(0, Constants.shooter_kI);
      masterTalon.config_kD(0, Constants.shooter_kD);
      masterTalon.config_kF(0, Constants.shooter_kF);
    }
    public void setFlywheelSpeed(double speed){
      masterTalon.set(ControlMode.Velocity, speed);
    }
    public void flywheelSpeedPercent(double speed){
      masterTalon.set(ControlMode.PercentOutput,speed);
    }
    public void reconfigureTalonGains(){
      masterTalon.config_kP(0, Constants.shooter_kP);
      masterTalon.config_kI(0, Constants.shooter_kI);
      masterTalon.config_kD(0, Constants.shooter_kD);
      masterTalon.config_kF(0, Constants.shooter_kF);
    }
    public double getFlywheelVelocity(){
      return masterTalon.getSelectedSensorVelocity();
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
