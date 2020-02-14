/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.commands.auto.actions.AimAndShoot;
import frc.robot.commands.auto.paths.TestPath;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class TestAuto extends AutoBase{
    public TestAuto(DriveTrain dt){
        addAction(new TestPath(dt), Timing.SEQUENTIAL);
        addAction(new AimAndShoot(), Timing.SEQUENTIAL);
    }
}
