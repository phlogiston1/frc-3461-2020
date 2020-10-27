/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Will add comments when thing is build (& code is tested)
   */
  DoubleSolenoid pto = new DoubleSolenoid(2,3);
  public Climber(DriveTrain dt) {
  }
  public void engagePTO(boolean engaged){
    if(engaged){
      pto.set(Value.kForward);
    }else{
      pto.set(Value.kReverse);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
