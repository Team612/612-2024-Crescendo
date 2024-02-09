// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
public class ClimbArm extends Command {
  private final Climb m_climb;

    //Constructor
    public ClimbArm(Climb climb){
        m_climb = climb;
        addRequirements(climb);
    }
    //Extend the pivot arm
    @Override
    public void initialize(){
        
    }

    //Retract pivot arm to pull robot to the rung
    @Override
    public void execute(){
        m_climb.extendArm();
    }
    
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}