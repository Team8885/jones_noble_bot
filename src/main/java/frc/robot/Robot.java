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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myDrive;
  private XboxController m_Stick;
  private Spark m_frontRightMotor;
  private Spark m_frontLeftMotor;
  private Spark m_rearRightMotor;
  private Spark m_rearLeftMotor;
  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;
  private ADXRS450_Gyro m_gyro;
  private Timer m_timer;
  private Encoder m_encoderLeft;
  private Encoder m_encoderRight;

  private static double kAngleSetpoint = 0.0;  // turn angle, relative to start 
	private static final double kP = 0.005;      // propotional turning constant

  @Override
  public void robotInit() {
  
    m_frontLeftMotor  = new Spark(0);
    m_rearLeftMotor   = new Spark(1);
    m_frontRightMotor = new Spark(2);
    m_rearRightMotor  = new Spark(3);
    m_leftMotors      = new MotorControllerGroup(m_frontLeftMotor,  m_rearLeftMotor);
    m_rightMotors     = new MotorControllerGroup(m_frontRightMotor, m_rearRightMotor);
    m_myDrive         = new DifferentialDrive   (m_leftMotors,      m_rightMotors);
    m_gyro            = new ADXRS450_Gyro();
    m_timer           = new Timer();
    m_Stick           = new XboxController(0);
    m_encoderLeft     = new Encoder(0,1);
    m_encoderRight    = new Encoder(2,3);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. For our gearbox and chassis 
    // orientation, we need to invert the left side.
    m_rightMotors.setInverted(true);

    // calibrate the gyro, assumes robot is stationary, facing where you want
    m_gyro.calibrate();

    // initialize the left and right encoders
    m_encoderLeft. setDistancePerPulse(4./256.);   // encoder returns 4 for every 256 pulses
    m_encoderRight.setDistancePerPulse(4./256.);
    m_encoderLeft. setMaxPeriod(.1);               // encoder stopped after .1 seconds
    m_encoderRight.setMaxPeriod(.1); 
    m_encoderLeft. setMinRate(10);                 // encoder stopped when its rate is below 10
    m_encoderRight.setMinRate(10);
    m_encoderLeft. setSamplesToAverage(5);         // encoder averages 5 samples
    m_encoderRight.setSamplesToAverage(5);
    m_encoderLeft. setReverseDirection(true);      // encoder direction reversed

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
     m_leftMotors.set(-0.4);
     m_rightMotors.set(-0.4);
     m_myDrive.feed();
    } else {
      m_leftMotors.set(0.0);
      m_rightMotors.set(0.0);
      m_myDrive.feed();
  }
}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double speed = -m_Stick.getRawAxis(1);//*0.6;
    //double turn  = m_Stick.getRawAxis(4);//*0.3;

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
