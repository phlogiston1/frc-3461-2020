/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.actions.Action;

/**
 * Add your docs here.
 */
public class AutoBase extends CommandBase{
    List<Action> actions = new ArrayList<Action>();
    List<Object> timing = new ArrayList<Object>();
    private boolean active;
    protected double mUpdateRate = 1.0 / 50.0;
    public AutoBase(){
        
    }
    public void addAction(Action action, Timing time) {
        actions.add(action);
        timing.add(timing);
    }
    public void run(){
        active = true;
        for(int i = 0; i < actions.size(); i++){
            Action action = actions.get(i);
            runAction(action);
            if(timing.get(i) == Timing.SEQUENTIAL){
                while(!action.isFinished()){}
            }
        }
        active = false;
    }
    public void stop(){
        active = false;
    }
    public boolean isActive(){
        return active;
    }
    public void done(){
        System.out.println("Auto done");
    }
    public void runAction(Action action){
        action.start();
        while(isActive() && !action.isFinished()){
            action.update();
            long waitTime = (long) (mUpdateRate * 1000.0);
            try{
                Thread.sleep(waitTime);
            }catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    public enum Timing{
        SEQUENTIAL,
        PARRALEL;
    }
}
