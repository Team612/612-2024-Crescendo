// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class ControlMap {
    private static int DRIVER_PORT = 0;
    private static int GUNNER_PORT = 1;
    private static int DRIVER_PORT_BUTTONS = 2;
    private static int GUNNER_PORT_BUTTONS = 3;
    
    //Controller objects
    public static Joystick driver_joystick = new Joystick(DRIVER_PORT);
    public static Joystick gunner_joystick = new Joystick(GUNNER_PORT);
    public static Joystick driver_buttons = new Joystick(DRIVER_PORT_BUTTONS);
    public static Joystick gunner_buttons = new Joystick(GUNNER_PORT_BUTTONS);


    public static JoystickButton blue1 = new JoystickButton(gunner_buttons, 1);
    public static JoystickButton blue2 = new JoystickButton(gunner_buttons, 2);
    public static JoystickButton green2 = new JoystickButton(gunner_buttons, 3);
    public static JoystickButton red4 = new JoystickButton(gunner_buttons, 4);
    public static JoystickButton red5 = new JoystickButton(gunner_buttons, 5);
    public static JoystickButton red6 = new JoystickButton(gunner_buttons, 6);
    public static JoystickButton yellow1 = new JoystickButton(driver_buttons, 1);
    public static JoystickButton yellow2 = new JoystickButton(driver_buttons, 2);
    public static JoystickButton green1 = new JoystickButton(driver_buttons, 3);
    
    public static JoystickButton red1 = new JoystickButton(driver_buttons,6);
    public static JoystickButton red2 = new JoystickButton(driver_buttons,5);
    public static JoystickButton red3 = new JoystickButton(driver_buttons,4);
    public static JoystickButton GUNNER_LB = new JoystickButton(gunner_joystick, 5); //LB
    public static JoystickButton GUNNER_RB = new JoystickButton(gunner_joystick, 6); //RB
}
