/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.actions;

import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.Limelight;
import frc.robot.commands.AutoAim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

/**
 * Add your docs here.
 */
public class AimAndShoot implements Action{ //todo
    private Turret turret;
    private DriveTrain driveTrain;
    private Limelight camera;
    private AutoAim aimer;
    public AimAndShoot(Turret t, DriveTrain dt, Limelight cam, RobotState state){
        turret = t;
        driveTrain = dt;
        camera = cam;
        aimer = new AutoAim(turret, driveTrain, camera, state);
    }
    public boolean isFinished(){
        return false;
    }
    public void update(){

    }
    public void done(){
        CommandScheduler.getInstance().cancel(aimer);
    }
    public void start(){
        CommandScheduler.getInstance().schedule(aimer);
    }
}
