/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myDrive;
  private XboxController m_joyStick;
  private Spark m_leftMotor;
  private Spark m_rightMotor;

  @Override
  public void robotInit() {
  
    m_leftMotor  = new Spark(0);
    m_rightMotor = new Spark(2);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. For our gearbox and chassis 
    // orientation, we need to invert the left side.
    m_leftMotor.setInverted(true);
    m_myDrive    = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_joyStick   = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
    // We introduced a damping factor because full-power made the robot
    // erratic and crab-like, due to unequal power from each side at
    // maximum joystick travel.  The damping factor of 0.3 resulted in
    // NO movement, 0.6 was a slow crawl, and 0.7 to 1.0 was the sweet
    // spot.  With both sides at 0.7, the robot slowly pulls to the right
    // moving away, but "calibration" will have to wait for some time
    // on the Provo High carpeted field (in the wrestling gym).
    m_myDrive.tankDrive(m_joyStick.getLeftY()*0.85, m_joyStick.getRightY()*0.85);
  }
}
