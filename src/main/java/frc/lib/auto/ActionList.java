/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.auto;

import frc.robot.commands.auto.PathBase;
import frc.robot.commands.auto.actions.ActionBase;

/**
 * Add your docs here.
 */
public class ActionList {
    private PathBase[] paths;
    private ActionBase[] actions;
    private boolean[][] pathActionSwitch;
    private int pathActionSwitchNextAddIndex = 0;
    private int pathNextIndex = 0;
    private int actionNextIndex = 0;
    private PathBase nextPath;
    private ActionBase nextAction;
    public ActionList(){

    }
    public void addAction(ActionBase action, ExecutionType execution){
        
    }
    public void addPath(PathBase path, ExecutionType execution){
        
    }
    public void addToArray(PathBase path){
        PathBase[] newArray = new PathBase[paths.length + 1];
        for(int i = 0; i < paths.length + 1; i++){
            if(i < paths.length){
                newArray[i] = paths[i];
            }else{
                newArray[i] = path;
            }
        }
    }
    public void addToArray(ActionBase action){
        ActionBase[] newArray = new ActionBase[actions.length + 1];
        for(int i = 0; i < actions.length + 1; i++){
            if(i < actions.length){
                newArray[i] = actions[i];
            }else{
                newArray[i] = action;
            }
        }
    }
    public enum ExecutionType{
        PARALLEL,
        SEQUENTIAL;
    }

}