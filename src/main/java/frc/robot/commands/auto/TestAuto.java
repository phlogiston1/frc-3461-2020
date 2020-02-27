/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.io.IOException;

import frc.robot.RobotState;
import frc.lib.Limelight;
import frc.robot.commands.auto.actions.AimAndShoot;
//import frc.robot.commands.auto.actions.AimAndShoot;
import frc.robot.commands.auto.paths.TestPath;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

/**
 * A file to test the auto framework.
 */
public class TestAuto extends AutoBase {
    /**
     * add actions to the stack. call run on this to start auto.
     *
     * @param dt the drive train to run the ramsete command on.
     * @throws IOException
     */
    public TestAuto(DriveTrain dt, Limelight cam, Turret t, RobotState s) throws IOException {
        addAction(new TestPath(dt), Timing.SEQUENTIAL);//TODODODODO
        addAction(new AimAndShoot(t,dt,cam,s), Timing.SEQUENTIAL);
    }
}
