// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;

public class LED extends SubsystemBase {
  private static AddressableLED m_led;
  private static LED m_LED;
  private AddressableLEDBuffer m_ledBuffer;
  public Vision m_Vis;
  Random rand = new Random();
  /** Creates a new Led_Colors. */
  public LED() {
      m_Vis = new Vision(null, null);
      m_led = new AddressableLED(0);
      m_LED = new LED();
      m_ledBuffer = new AddressableLEDBuffer(300); //300 should be the length
      m_led.setLength(m_ledBuffer.getLength());
      m_led.start();
  }

  public static LED getInstance(){
    return m_LED;
  }

  public void setOneLed(int l, int r, int g, int b){
    m_ledBuffer.setRGB(l, r, g, b);
    m_led.setData(m_ledBuffer);
  }

  public void setAllLeds(int r, int g, int b){
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, r, g, b);
      m_led.setData(m_ledBuffer);
    }
  }

  public void setGreen(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 0, 255, 0);
      m_led.setData(m_ledBuffer);
    }
  }

  public void setPink(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 255,20,147);
      m_led.setData(m_ledBuffer);
    }
  }

    public void setGreenAndPink(){
      boolean pink = true;
      int a = 0;
      for (int i = 0; i < m_ledBuffer.getLength(); i++){
        if(pink == true){
          a++;
          m_ledBuffer.setRGB(i, 255,20,147);
          m_led.setData(m_ledBuffer);
          if(a == 10){
            a=0;
            pink = false;
          }
        }
        else{
          a++;
          m_ledBuffer.setRGB(i, 0, 255, 0);
          m_led.setData(m_ledBuffer);
          if(a == 10){
            a=0;
            pink = true;
          }
        }
      }
    }

  public void setPurpleAndWhite(){
    boolean purple = true;
    int a = 0;
    
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      if(purple == true){
        a++;
        m_ledBuffer.setRGB(i,148,0,211);
        m_led.setData(m_ledBuffer);
        if(a == 10){
          a=0;
          purple = false;
        }
      }
      else{
        a++;
        m_ledBuffer.setRGB(i, 255, 255, 255);
        m_led.setData(m_ledBuffer);
        if(a == 10){
          a=0;
          purple = true;
        }
      }
    }
  }
  
  // length
  public int getLength(){
    return m_ledBuffer.getLength();
  }


  // Special Effects
  public void fireAnimation(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 255, rand.nextInt(255)+1, 0);
      m_led.setData(m_ledBuffer);
    }
  }



  @Override
  public void periodic() {
        //Default (Not detecting anything):
          // 612 Colors/Chantilly Colors: Blue & Yellow, Purple & White
        // In Robot Container
        
        
        // When Detects Only April Tags:
            // Green
        


        // When Sees Only Note:
            // Pink 



        // When sees note & april tags:
            // Green & Pink: Alternating green and pink around in increments of 3
  }
}
