/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class DriveOffLine extends PathBase {

    public DriveOffLine(DriveTrain subsystem) {
        super(subsystem);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0,new Rotation2d(0)),
            List.of(
                new Translation2d(0.5,0)
            ),
            new Pose2d(1,0, new Rotation2d(0)),
            getTrajectoryConfig()
        );
        setTrajectory(trajectory);
    }
}
