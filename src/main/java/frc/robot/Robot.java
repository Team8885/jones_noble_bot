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
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myDrive;
  private XboxController m_Stick;
  private Spark m_rightMotor;
  private Spark m_leftMotor;
  //private Spark m_frontRightMotor;
  //private Spark m_rearRightMotor;
  //private MotorControllerGroup m_leftGroupMotor;
  //private MotorControllerGroup m_rightGroupMotor;
  private ADXRS450_Gyro m_gyro;
  private Timer m_timer;

  private static double kAngleSetpoint = 0.0;  // turn angle, relative to start 
	private static final double kP = 0.005;      // propotional turning constant

  @Override
  public void robotInit() {
  
    m_leftMotor  = new Spark(0);
    m_rightMotor   = new Spark(1);
    //m_frontRightMotor = new Spark(2);
    //m_rearRightMotor  = new Spark(3);
    m_timer = new Timer();
    //leftMotor  = new MotorControllerGroup(m_frontLeftMotor,  m_rearLeftMotor);
    //m_rightMotor = new MotorControllerGroup(m_frontRightMotor, m_rearRightMotor);
    m_gyro            = new ADXRS450_Gyro();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. For our gearbox and chassis 
    // orientation, we need to invert the left side.
    //m_rightGroupMotor.setInverted(true);
    m_rightMotor.setInverted(true);
    m_myDrive    = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_Stick      = new XboxController(0);

    // calibrate the gyro, assumes robot is stationary, facing where you want
    m_gyro.calibrate();

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_gyro.reset();   // set to zero angle, remove any drift since boot-up
  }

  @Override
  public void autonomousPeriodic() {
    if (m_timer.get() < 4.0) {
     m_leftMotor.set(-0.4);
     m_rightMotor.set(-0.4);
     m_myDrive.feed();
    } else {
      m_leftMotor.set(0.0);
      m_rightMotor.set(0.0);
      m_myDrive.feed();
  }
}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double speed = -m_Stick.getRawAxis(1);//*0.6;
    double turn  = m_Stick.getRawAxis(4);//*0.3;

    //double left  = speed + turn;
   //double right = speed - turn;

    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP*10.0;
		// Invert the direction of the turn if we are going backwards
		turningValue = Math.copySign(turningValue, speed);
		//m_myDrive.arcadeDrive(-speed, turningValue);
     //m_myDrive.arcadeDrive(speed, turn);
     m_myDrive.arcadeDrive(speed, turningValue);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

}
