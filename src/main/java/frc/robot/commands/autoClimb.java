// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
public class autoClimb extends Command {
  private final Climb m_climb;
    private final Drivetrain m_drivetrain;
    //Constructor
    public autoClimb(Drivetrain drive, Climb climb){
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
                m_climb.Oneset();
            }
            else if (m_drivetrain.getRoll() < -10){
                m_climb.Twoset();
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