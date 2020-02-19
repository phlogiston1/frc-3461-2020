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
 * Complete auto framework. An auto is built by adding actions to a stack. Then call run to start and
 * run each action from the top of the stack to the bottom.
 * runAction() from team 254.
 */
public class AutoBase extends CommandBase{
    private List<Action> actions = new ArrayList<Action>();
    private List<Timing> timing = new ArrayList<Timing>();
    private boolean active;
    protected double mUpdateRate = 1.0 / 50.0;
    public AutoBase(){
    }
    /**
     * add an action to the stack
     * @param action The action to add
     * @param time Timing to tell whether to run with the next action, or before.
     */
    public void addAction(Action action, Timing time) {
        actions.add(action);
        timing.add(time);
    }
    /**
     * run the stack.
     */
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
    /**
     * stop running
     */
    public void stop(){
        active = false;
    }
    /**
     * is auto running
     * @return boolean;
     */
    public boolean isActive(){
        return active;
    }
    /**
     * print "Auto done"
     */
    public void done(){
        System.out.println("Auto done");
    }
    /**
     * team 254's runAction. starts and updates an action until it is finished or stopped.
     * @param action the action to run
     */
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
        /**
         * enum for whether to run actions sequentially or in parallel/
         */
        SEQUENTIAL,
        PARALLEL;
    }
}
