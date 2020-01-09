/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4573.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;




public class Robot extends TimedRobot {
	 
	Joystick drive_stick; 
	Joystick control_stick;
	DoubleSolenoid climbPiston;
	DoubleSolenoid grabberPiston;
	DifferentialDrive m_drive;
	ADXRS450_Gyro gyro;
	DigitalInput limitSwitch;
	Timer timer;
	
	double m_deadZone;
	double m_driveMotorSpeed;
	double m_autoDriveSpeed;
	
	int displayCtr; 
	
	int currentState; 
	int nextState;
	double waitTime;
	double speed;
	int direction;
	double angle;
	double aTurnAngle; // auto turn angle
	
	static final int IMG_WIDTH = 320;
	static final int IMG_HEIGHT = 240;
	
	static final int LEFT = -1;
	static final int RIGHT = 1; 
	static final int WAITSTATE = 0;
	static final int DRIVESTATE = 1;
	static final int LTURNSTATE = 2;
	static final int RTURNSTATE = 3;
	static final int ENDSTATE = 10;
	
	String autoSeq;
	char seqState;
	int seqValue;
	int seqIndex;
	
     //*****************************************************************************
     //* This function is run when the robot is first started up and should be
     //* used for any initialization code.
	 //*****************************************************************************
	@Override
	public void robotInit() {
		
		m_deadZone = 0.1;
		m_driveMotorSpeed = 1.0;
		m_autoDriveSpeed = 0.5;
		displayCtr = 0;
		seqIndex = 0;
		drive_stick = new Joystick(0);
	    control_stick = new Joystick(1);
        climbPiston = new DoubleSolenoid(0,1); // Cylinder solenoid ch. 0/1 for Climbing Piston
        grabberPiston = new DoubleSolenoid(2,3); // Cylinder solenoid ch. 2/3 for Grabber Piston
		
		Spark m_rearRight = new Spark(0);
		Spark m_frontRight = new Spark(1);
		Spark m_frontLeft = new Spark(2);
		Spark m_rearLeft = new Spark(3);

		m_frontLeft.setSafetyEnabled(false);  
		m_rearLeft.setSafetyEnabled(false);
		m_frontRight.setSafetyEnabled(false);  
		m_rearRight.setSafetyEnabled(false);
		
		SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
		SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
		
	    m_drive = new DifferentialDrive(m_left, m_right);
	    
	    gyro = new ADXRS450_Gyro(); // Digital Gyro on SPI interface of RoboRIO
	    gyro.calibrate(); // Calibrate Gyro on start

	    
	    UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture("DriveCam", "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0");
		camera0.setResolution(IMG_WIDTH/2, IMG_HEIGHT/2);
		camera0.setFPS(15); 
		
		limitSwitch = new DigitalInput(0);
		 
		timer = new Timer();
		currentState = ENDSTATE;
		seqValue = 0;
		seqState = 'E';
		autoSeq = "W0L2E0";
		SmartDashboard.putString("DB/String 0","W0L2E0");
		updateDisplays();
	}

	 //******************************************************************
	 // this routine is called 1 time at the beginning of autonomous mode
	 //******************************************************************
	
	@Override
	public void autonomousInit() {
		
		gyro.reset();  // Reset Gyro at start of Autonomous
		/*		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage(); //Read game data from FMS
		
		 if(gameData.charAt(0) == 'L')  // First char indicates alliance switch owner position
		{
			SmartDashboard.putString("DB/String 0", " Left "); //  
		} else {
			SmartDashboard.putString("DB/String 0", "Right"); //  
		} updateDisplays();	 */
		

		
		
		
		autoSeq = SmartDashboard.getString("DB/String 0","E0");
		seqIndex = 0;
		getNextState();
		updateDisplays();

		
	}
	

	 //********************************************************************
	 // this routine is called every 20 milliseconds during autonomous mode
	 //********************************************************************
	
	@Override
	public void autonomousPeriodic() {
		
	
		updateDisplays();	
		
		switch(currentState) {
			case WAITSTATE: {
				if(timer.get()> waitTime) {
					getNextState();
					
				}
				break;
			}
			case DRIVESTATE: {
				if(timer.get()< waitTime) {
		 			m_drive.arcadeDrive(0.5, -gyro.getAngle()*.08);
		 		}
				else {
					m_drive.arcadeDrive(0, 0); // stop driving
					getNextState();
				}
				break;
			}
			case LTURNSTATE: {
				if(gyro.getAngle() > aTurnAngle) {
					m_drive.arcadeDrive(0, -m_autoDriveSpeed);
				}	
				else {
					m_drive.arcadeDrive(0, 0); // stop driving
					getNextState();
				}
				break;
			}
			case RTURNSTATE: {
				if(gyro.getAngle() < aTurnAngle) {
					m_drive.arcadeDrive(0, m_autoDriveSpeed);
				}
				else {
					m_drive.arcadeDrive(0, 0);
					getNextState();
				}
				break;
			}
			case ENDSTATE: {
				m_drive.arcadeDrive(0, 0); // stop driving
				break;
			}
		}
	}

	 //***************************************************************
	 // this routine is called 1 time at the beginning of teleop mode
	 //***************************************************************
		
	@Override
	public void teleopInit() {
		updateDisplays();		
		}

	 //*********************************************************
	 // this is called every 20 milliseconds during teleop mode
	 //*********************************************************
	
	@Override
	public void teleopPeriodic()
	{
		// Get Joystick input for arcade driving
		double X = getJoystickValue(drive_stick, 1) * m_driveMotorSpeed; 
        double Z = getJoystickValue(drive_stick, 2) * m_driveMotorSpeed; 
       
        m_drive.arcadeDrive(-X, Z, true); // Drive the robot

        
        if (limitSwitch.get()==true) {
        	
        }	
     
    	if (control_stick.getRawButton(1)==true) { 
			climbPiston.set(DoubleSolenoid.Value.kForward);
		}
		if (control_stick.getRawButton(4)==true) { 
			climbPiston.set(DoubleSolenoid.Value.kReverse);
		} 
		if (control_stick.getRawButton(3)==true) {
			grabberPiston.set(DoubleSolenoid.Value.kForward);
		}
		if (control_stick.getRawButton(2)==true) {
			grabberPiston.set(DoubleSolenoid.Value.kReverse);
		}
		updateDisplays();	
    
  
	}

	 //*******************************************************
	 // This function is called periodically during test mode.
	 //******************************************************* 
	
	@Override
	public void testPeriodic() {
	}
	
	 //*******************************************************************
	 // This function is used to read joystick & eliminate deadzone issues
	 //*******************************************************************
	
	public double getJoystickValue(Joystick joystick, int iKey) {
    	double dVal = joystick.getRawAxis(iKey);
        if (Math.abs(dVal) < m_deadZone) {
            return 0;
        } else {
            return dVal;
        }
    }

	 // ***************************************************************************
	 // This function displays info on the Labview Smartdashboard periodically
     // ***************************************************************************

		public void updateDisplays() {
			if (displayCtr % 25 == 0){	// Update displays on Dashboard every ~500msec
 
				SmartDashboard.putString("DB/String 2", Double.toString(currentState)); //   
				SmartDashboard.putString("DB/String 3", autoSeq);
	    	   	SmartDashboard.putString("DB/String 6", Double.toString(Math.round(gyro.getAngle() * 1d)/1d ) + " Deg.");  // round to 0 decimal places
	     	 	SmartDashboard.putString("DB/String 8", Character.toString(seqState) + " State");
	            SmartDashboard.putString("DB/String 9", Double.toString(seqValue));  // 
		 	}
			displayCtr++;
		}
		/* this function parses the auto sequence string from the dashboard */
		
		public void getNextState() 	{
			seqState = autoSeq.charAt(seqIndex); // read sequence command
			seqIndex++;
			seqValue = Character.getNumericValue(autoSeq.charAt(seqIndex)); // read sequence value as an integer
			seqIndex++;
			if((seqValue < 0 ) || (seqValue > 9)) {
				seqValue = 0;
			}
			
			switch (seqState) {
				case 'W': {
					currentState = WAITSTATE;
					waitTime = seqValue;
					timer.reset();
					timer.start();
					break;
				}
				case 'D': {
					currentState = DRIVESTATE;
					waitTime = seqValue;
					timer.reset();
					timer.start();
					gyro.reset();
					break;
			    }
				
				case 'L': { 
					currentState = LTURNSTATE;
				    aTurnAngle = -42 * seqValue; // auto turn angle is an increment of -45 degrees
				    gyro.reset();
				    break;
				}
				case 'R': {
					currentState = RTURNSTATE;
					aTurnAngle = 42 * seqValue; //  auto turn angle is an increment of 45 degrees
					gyro.reset();
					break; 
				}
				default:
				case 'E': {
					currentState = ENDSTATE;
					timer.reset();
					break;
				}
			}
			
		}
	
}    // End all
	