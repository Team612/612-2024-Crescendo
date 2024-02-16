// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.photonvision.PhotonCamera;
import frc.robot.subsystems.Drivetrain;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import org.photonvision.PhotonUtils;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Limelight;

public class driveToClimb extends Command {
  private Drivetrain m_drivetrain;
  private Limelight m_vision;
  private double speed = 0.3;
  private int timer = 0;
  private boolean runOnce = false;
  private double range = 0.0;
  private double circumference = 1;
  boolean done = false;
  Translation2d translation2d;
  
  //private SwerveModule m_swerve;
  
  /** Creates a new DriveToObject. */
  public driveToClimb(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drivetrain.getPositionsDist()*circumference < 1) {
      m_drivetrain.drive(
        new Translation2d(1, 0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        true);
    }
    else{
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
