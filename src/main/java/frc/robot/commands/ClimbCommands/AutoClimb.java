// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
public class AutoClimb extends Command {
  private final Climb m_climb;
    private final Drivetrain m_drivetrain;
    //Constructor
    public AutoClimb(Drivetrain drive, Climb climb){
        m_climb = climb;
        m_drivetrain = drive;
        addRequirements(m_climb, m_drivetrain);
    }
    //Extend the pivot arm
    @Override
    public void initialize(){
        m_climb.freezeMotors();
    }

    //Retract pivot arm to pull robot to the rung
    @Override
    public void execute(){
            if (m_drivetrain.getRoll() > 10){
                m_climb.setSpeed(1,0);
            }
            else if (m_drivetrain.getRoll() < -10){
                m_climb.setSpeed(0,1);
            }
            else{
                m_climb.freezeMotors();
            }
    }
    
    
    @Override
    public void end(boolean interrupted){
        m_climb.freezeMotors();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}