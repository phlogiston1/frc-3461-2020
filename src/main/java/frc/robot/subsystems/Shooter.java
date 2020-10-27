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
 * simple shooter code with velocity control
 * FLYWHEEL ONLY. Hood and turret are controlled in 'Turret.java'
 */
public class Shooter extends SubsystemBase {
    //create the motors
    private TalonFX masterTalon = new TalonFX(Constants.SHOOTER_MOTOR_1);
    private TalonFX slaveTalon = new TalonFX(Constants.SHOOTER_MOTOR_2);

    /**
     * creates a new shootah
     */
    public Shooter() {
      //follow motors
      slaveTalon.follow(masterTalon);
      masterTalon.setInverted(false); //TODO make sure motor is spinning forwards
      slaveTalon.setInverted(InvertType.FollowMaster); //TODO make sure motors aren't fighting

      //set pidf constants
      masterTalon.config_kP(0, Constants.shooter_kP);
      masterTalon.config_kI(0, Constants.shooter_kI);
      masterTalon.config_kD(0, Constants.shooter_kD);
      masterTalon.config_kF(0, Constants.shooter_kF);
    }

    /**
     * set the speed of the flywheel using falcon pidf velocity control
     * @param speed velocity in (position change(of encoders))/100ms
     */
    public void setFlywheelSpeed(double speed){
      masterTalon.set(ControlMode.Velocity, speed);
    }

    /**
     * simple, set power output as percent of total
     * DONT GO NEGATIVE if I did 'setInverted' correctly
     * @param speed power as (-1 to 1)
     */
    public void flywheelSpeedPercent(double speed){
      masterTalon.set(ControlMode.PercentOutput,speed);
    }

    /**
     * dont know if this is ever used, reset pidf gains to the values in Constants
     */
    public void reconfigureTalonGains(){
      masterTalon.config_kP(0, Constants.shooter_kP);
      masterTalon.config_kI(0, Constants.shooter_kI);
      masterTalon.config_kD(0, Constants.shooter_kD);
      masterTalon.config_kF(0, Constants.shooter_kF);
    }

    //self explanatory
    public double getFlywheelVelocity(){
      return masterTalon.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
