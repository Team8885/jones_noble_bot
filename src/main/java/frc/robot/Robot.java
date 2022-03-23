/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myDrive;
  private XboxController m_joyStick;
  private Spark m_leftMotor;
  private Spark m_rightMotor;

  @Override
  public void robotInit() {
  
    m_leftMotor  = new Spark(0);
    m_rightMotor = new Spark(2);
    m_leftMotor.setInverted(true);
    m_myDrive    = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_joyStick   = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
    m_myDrive.tankDrive(m_joyStick.getLeftY(), m_joyStick.getRightY());
  }
}
