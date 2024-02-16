package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import frc.robot.subsystems.Drivetrain;

public class Climb extends SubsystemBase {
    //defines both Talons and Solenoids
    private final DoubleSolenoid secondClimb;
    private final DoubleSolenoid climb;
    
    public Climb(){
        secondClimb = new DoubleSolenoid(Constants.VisionConstants.PCM_2, Constants.VisionConstants.solenoidType, Constants.VisionConstants.firstSolenoid[0], Constants.VisionConstants.firstSolenoid[1]);
        climb  = new DoubleSolenoid(Constants.VisionConstants.PCM_2, Constants.VisionConstants.solenoidType, Constants.VisionConstants.secondSolenoid[0], Constants.VisionConstants.secondSolenoid[1]);
    }
    //Pushes the piston out
    public void extendArm(){
        climb.set(Value.kForward);
        secondClimb.set(Value.kForward);
    }

    //partial extention
    public void partialExtention(){
        climb.set(Value.kForward);
        secondClimb.set(Value.kReverse);
    }
    
    //Brings the piston in
    public void retractArm(){
        climb.set(Value.kReverse);
        secondClimb.set(Value.kOff);
    }

    public void Oneset(){
        climb.set(Value.kForward);
    }

    public void Twoset(){
        secondClimb.set(Value.kForward);
    }

    public void freezeMotors(){
        climb.set(Value.kOff);
        secondClimb.set(Value.kOff);
    }



    // public double getRoll(){
    //     return navx.getRoll();
    //   }

    //Pivots the pivot arms


    @Override
    public void periodic(){
        
    }
}

// Copyright (c) FIRST and other WPILib contributors. // Open Source Software; you can modify and/or share it under the terms of // the WPILib BSD license file in the root directory of this project. package frc.robot.subsystems; import edu.wpi.first.wpilibj2.command.SubsystemBase; import edu.wpi.first.wpilibj.DoubleSolenoid; import edu.wpi.first.wpilibj.DoubleSolenoid.Value; import frc.robot.Constants; import com.kauailabs.vmx.AHRSJNI; import com.kauailabs.navx.frc.AHRS; import edu.wpi.first.wpilibj.I2C; // import frc.robot.Robot; // import frc.robot.commands.ClimbArm; public class Climb extends SubsystemBase { private final DoubleSolenoid climb; private final DoubleSolenoid secondClimb;  //Pushes the piston out private static AHRS navx1; private static AHRS navx2; private double navxM1 = navx1.getRoll(); private double navxM2 = navx2.getRoll(); /** Creates a new Climb. */ public Climb() { climb = new DoubleSolenoid(Constants.VisionConstants.PCM_2, Constants.VisionConstants.solenoidType, Constants.VisionConstants.firstSolenoid[1], Constants.VisionConstants.firstSolenoid[0]); secondClimb = new DoubleSolenoid(Constants.VisionConstants.PCM_2, Constants.VisionConstants.solenoidType, Constants.VisionConstants.secondSolenoid[1] ,Constants.VisionConstants.secondSolenoid[0]); navx1 = new AHRS(I2C.Port.kMXP); navx2 = new AHRS(I2C.Port.kMXP); } public void extendArm(){ climb.set(Value.kForward); secondClimb.set(Value.kForward); } //partial extention public void partialExtention(){ climb.set(Value.kForward); secondClimb.set(Value.kReverse); } //Brings the piston in public void retractArm(){ climb.set(Value.kReverse); secondClimb.set(Value.kOff); } public void autoClimb(){ if(navxM2 > navxM1){ climb.set(Value.kForward); } if(navxM1 > navxM2){ } } public void freezeMotors(){ climb.set(Value.kOff); secondClimb.set(Value.kOff); System.out.println("STOP"); } public void climb() { extendArm(); retractArm(); } @Override public void periodic() { // This method will be called once per scheduler run } }