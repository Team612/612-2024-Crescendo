// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AlignToShooter_Detected extends CommandBase {
  /** Creates a new SpeakerAlign. */
  Drivetrain m_romi;
  double absolutePos = 0;
  boolean done = false;
  Vision m_poto;
  public AlignToShooter_Detected(Drivetrain romi, Vision poto) {
    m_romi = romi;
    m_poto = poto;
    addRequirements(m_poto);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  //If see april
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_poto.hasTagPose());
    if(m_poto.getTagYaw() < 0){
      Translation2d e = new Translation2d(0, 0.4);
      m_romi.drive(e,0.4,true);;
    }
    else{
      Translation2d e = new Translation2d(0, -0.4);
      m_romi.drive(e,-0.4,true);;
    }

    if(m_poto.getTagYaw()==1){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (done){
      Translation2d e = new Translation2d(0, 0.0);
      m_romi.drive(e,0.0,true);;
    }
    return done;
  }
}
