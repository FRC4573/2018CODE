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
import edu.wpi.first.wpilibj.Talon;

public class Robot extends TimedRobot {
	 
	Joystick drive_stick; 
	Joystick control_stick;
	DoubleSolenoid climbPiston;
	DoubleSolenoid grabberPiston;
	DoubleSolenoid drawerPiston;
	DifferentialDrive m_drive;
	ADXRS450_Gyro gyro;
	DigitalInput limitSwUpper;
	DigitalInput limitSwLower;
	Timer timer;
	Talon m_elevator;
	double m_deadZone;
	double m_driveMotorSpeed;
	double m_driveTurnSpeed;
	double m_autoDriveSpeed;
	double m_autoTurnSpeed;
	double m_elevatorSpeed;
	
	int displayCtr; 
	int currentState; 
	double waitTime;
	double aTurnAngle; // auto turn angle
	boolean climbingFlag; // climbing flag
	
	static final int IMG_WIDTH = 320;
	static final int IMG_HEIGHT = 240;
	static final int WAITSTATE = 0;
	static final int DRIVESTATE = 1;
	static final int LTURNSTATE = 2;
	static final int RTURNSTATE = 3;
	static final int ENDSTATE = 10;
	static final boolean ACTIVE = false; // limit switches are active low (false)
	static final boolean INACTIVE = true; 
    // control joystick buttons for pistons
	static final int EXT_DRAWER = 6;
	static final int RET_DRAWER = 7;
	static final int EXT_CLIMBER = 8;
	static final int RET_CLIMBER = 9;
	static final int CLOSE_GRABBER = 1;
	static final int OPEN_GRABBER = 3;
	
			
	String gameData;
	String autoMap; // string composed of robot starting position (L,C,R) and switch ownership side (L,R)
	String autoSeq;
	String autoTemp;
 
	char seqState;
	int seqValue;
	int seqIndex;
	
	int autoDelay;
	int autoProg;
	
     //*****************************************************************************
     //* This function is run when the robot is first started up and should be
     //* used for any initialization code.
	 //*****************************************************************************
	@Override
	public void robotInit() {
		
		m_deadZone = 0.1;
		m_driveMotorSpeed = 1.0;
		m_driveTurnSpeed =0.75;
		m_autoDriveSpeed = 0.60;
		m_autoTurnSpeed = 0.60;
		m_elevatorSpeed = 1.0;
		displayCtr = 0;
		seqIndex = 0;
		gameData="";
		drive_stick = new Joystick(0);
	    control_stick = new Joystick(1);
        climbPiston = new DoubleSolenoid(0,1); // Cylinder solenoid ch. 2/3 for Climbing Piston
        grabberPiston = new DoubleSolenoid(2,3); // Cylinder solenoid ch. 4/5 for Grabber Piston
        drawerPiston = new DoubleSolenoid(4,5); // Cylinder solenoid ch. 0/1 for Extending "Drawer" Piston
        limitSwUpper = new DigitalInput(9);
		limitSwLower = new DigitalInput(8);
		m_elevator = new Talon(5); //elevator motor controller
        
		Spark m_rearRight = new Spark(0);
		Spark m_frontRight = new Spark(1);
		Spark m_frontLeft = new Spark(2);
		Spark m_rearLeft = new Spark(3);
		
		SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
		SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
		
		m_elevator.setSafetyEnabled(false); 
	    
		m_drive = new DifferentialDrive(m_left, m_right);
	    
	    gyro = new ADXRS450_Gyro(); // Digital Gyro on SPI interface of RoboRIO
	    gyro.calibrate(); // Calibrate Gyro on start
	    
	    UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture("DriveCam",0);
		camera0.setResolution(IMG_WIDTH/2, IMG_HEIGHT/2);
		camera0.setFPS(15); 
		 
		timer = new Timer();
		currentState = ENDSTATE;
		seqValue = 0;
		seqState = 'E';
		autoSeq = "E0";
		SmartDashboard.putString("DB/String 5", "L");
		SmartDashboard.putString("DB/String 6", "0");
		SmartDashboard.putString("DB/String 7", "0");
		updateDisplays();
		
	}

	 //******************************************************************
	 // this routine is called 1 time at the beginning of autonomous mode
	 //******************************************************************
	
	@Override
	public void autonomousInit() {
		
		gyro.reset();  // Reset Gyro at start of Autonomous
		
		gameData = DriverStation.getInstance().getGameSpecificMessage(); //Read game data from FMS
		
		autoMap = SmartDashboard.getString("DB/String 5", "L");  // Default = Center if none selected
		autoTemp = SmartDashboard.getString("DB/String 6", "0");  // Default = 0 seconds if none entered
		autoDelay = Character.getNumericValue(autoTemp.charAt(0)); // convert first character of string to numeric value
		autoTemp = SmartDashboard.getString("DB/String 7", "0");  // Default = program 0 if none selected
		autoProg = Character.getNumericValue(autoTemp.charAt(0)); // Convert to numeric
		autoMap += Character.toString(gameData.charAt(0)); // autoMap now has 2 characters (starting position and switch ownership position)
	
		// 5Secs = 11ft
		switch(autoMap) {
			case "LL":{
				autoSeq = "W0D6R2D1E0";  // Tested  
				break;
			}
			case "LR":{
				autoSeq = "W0D2R2D9D3H1L2D4L2D1E0";// calculated
				if (autoProg == 1) {
					autoSeq = "W0D7W9E0";// For Prog 1 - Just break the autoline and wait
				}
				break;
			}
			case "CL":{
				autoSeq = "W0D2L2D5H1R2D4R2D1E0"; // Calculated
				break;
			}
			case "CR":{
				autoSeq = "W0D2R2D4L2D4L2D1E0"; //Calculated
				break;
			}
			case "RL":{
				autoSeq = "W0D2L2D9D3H1R2D4R2D1E0"; //Opposite of LR
				if (autoProg == 1) {
					autoSeq = "W0D7W9E0";// For Prog 1 - Just break the autoline and wait
				}
				break;
			}
			default:
			case "RR":{
				autoSeq = "W0D6L2D1E0";// Opposite of LL
			 
				break;	
			}
		}
	
		seqIndex = 0;
		getNextState();
		updateDisplays();
		
		// at start of auto close grabber and extend drawer
		closeGrabber();
		extendDrawer();
	}
	

	 //********************************************************************
	 // this routine is called every 20 milliseconds during autonomous mode
	 //********************************************************************
	
	@Override
	public void autonomousPeriodic() {
		
	
		updateDisplays();	
		
		if(limitSwUpper.get()==INACTIVE) {
			m_elevator.set(-1.0); // raise elevator while limit switch is inactive
		}
		else {
			m_elevator.set(0.0); // Stop elevator
		}
		
		switch(currentState) {
			case WAITSTATE:{
				m_drive.arcadeDrive(0, 0); // Make sure robot is stopped
				if(timer.get()> waitTime) {
					getNextState();
				}
				break;
			}
			case DRIVESTATE: {
				if(timer.get()< waitTime) {
		 			m_drive.arcadeDrive(m_autoDriveSpeed, -gyro.getAngle()*.08);
		 		}
				else {
					m_drive.arcadeDrive(0, 0); // stop driving
					getNextState();
				}
				break;
			}
			case LTURNSTATE: { 
				if(gyro.getAngle() > aTurnAngle) {//Turn Left to desired angle
					m_drive.arcadeDrive(0, -m_autoTurnSpeed);
				}	
				else {
					m_drive.arcadeDrive(0, 0); // stop driving
					getNextState();
				}
				break;
			}
			case RTURNSTATE: {
				if(gyro.getAngle() < aTurnAngle) {//Turn Right to desired angle
					m_drive.arcadeDrive(0, m_autoTurnSpeed);
				}
				else {
					m_drive.arcadeDrive(0, 0);
					getNextState();
				}
				break;
			}
			case ENDSTATE: {
				openGrabber();
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
		
		climbingFlag = false; // not climbing
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
        double Z = getJoystickValue(drive_stick, 2) * m_driveTurnSpeed; 
             
        m_drive.arcadeDrive(-X, Z, true); // Drive the robot

        double Y = getJoystickValue(control_stick, 1) * m_elevatorSpeed;
        
       if (((limitSwUpper.get()==ACTIVE) && (Y > 0))|| ((limitSwLower.get()==ACTIVE) && (Y < 0))){
          Y=0; // set speed to 0 if elevator tries to move past limit switche
          climbingFlag = false; // Reset climbing flag if limit switch is hit
       }	
       m_elevator.set(-Y); // Adjust elevator
 
       if (control_stick.getRawButton(RET_CLIMBER)==true) { 
			retractClimber();
		} 
		if (control_stick.getRawButton(EXT_CLIMBER)==true) { 
			climbingFlag = true; // enable flag to lower drawer
			extendClimber();
		}
		if ((control_stick.getRawButton(CLOSE_GRABBER)==true)) {
			closeGrabber();			
		}
		if (control_stick.getRawButton(OPEN_GRABBER)==true) {
			openGrabber();
		}
		if (control_stick.getRawButton(EXT_DRAWER)==true) {
			extendDrawer();
		}			
		if (control_stick.getRawButton(RET_DRAWER)==true) {
			retractDrawer();
		}
		if (climbingFlag){
			if(limitSwLower.get()==INACTIVE) {
				m_elevator.set(1.0); // Lower elevator while limit switch is inactive
			}
			else {
				climbingFlag = false; // Reset climbing flag if limit switch is hit
			}
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
	 //*******************************************************************
	 // These functions control the pneumatic pistons
	 //*******************************************************************
	public void closeGrabber(){
		grabberPiston.set(DoubleSolenoid.Value.kReverse);
	}
	public void openGrabber(){
		grabberPiston.set(DoubleSolenoid.Value.kForward);
	}
	public void extendDrawer(){
		drawerPiston.set(DoubleSolenoid.Value.kReverse);
	}	
	public void retractDrawer(){
		closeGrabber(); // ALWAYS close grabber before retracting drawer
		drawerPiston.set(DoubleSolenoid.Value.kForward);
	}
	public void extendClimber(){
		climbPiston.set(DoubleSolenoid.Value.kForward);
		retractDrawer(); // ALWAYS retract drawer when climbing
	}
	public void retractClimber(){
		climbPiston.set(DoubleSolenoid.Value.kReverse);
	}
	 // ***************************************************************************
	 // This function displays info on the Labview Smartdashboard periodically
     // ***************************************************************************

		public void updateDisplays() {
			if (displayCtr % 25 == 0){	// Update displays on Dashboard every ~500msec
				SmartDashboard.putString("DB/String 0", "StartingPos(L,C,R)");
				SmartDashboard.putString("DB/String 1", "StartDelay(secs)");
				SmartDashboard.putString("DB/String 2", "AutoProg(0 - 1)");
				SmartDashboard.putString("DB/String 3", autoSeq);
				SmartDashboard.putString("DB/String 4", gameData); 
	    	   	//SmartDashboard.putString("DB/String 5", "");
	    	   	//SmartDashboard.putString("DB/String 6","");  
	    	    //SmartDashboard.putString("DB/String 7", "");
	    	   	SmartDashboard.putString("DB/String 8", Character.toString(seqState) + " State");
	      //      SmartDashboard.putString("DB/String 9", Double.toString(seqValue));  // 
		 	}
			displayCtr++;
		}
		// ***************************************************************************
	    // This function parses the auto sequence string from the dashboard, and
		// Set variables up for the corresponding state.
		// The string can be any sequence of 2 character commands as follows:
		// Format of AutoCommands:
		// Wn = Wait n seconds ; IF n=0 autDelay over-ride value is used
		// Dn = Drive forward for n seconds: 5secs = 11ft
		// Lm = Turn Left m * 45 degrees 
		// Rm = Turn Right m * 45 degrees
		// E0 = End State - release (open) Grabber
		// where n and m must be between 0 and 9 inclusive
		//
		// Example: W0D2R2W3E0 = Wait 0 (use AutoDelay,  Drive Fwd 2 secs, Turn Right 90 Deg, Wait 3 secs, End 
	    // ***************************************************************************
		
		public void getNextState() 	{
			seqState = autoSeq.charAt(seqIndex); // read sequence command
			seqIndex++;
			seqValue = Character.getNumericValue(autoSeq.charAt(seqIndex)); // read sequence value as an integer
			seqIndex++;
			if((seqValue < 0 ) || (seqValue > 9)) {  //Make sure SeqValue is between 0 and 9
				seqValue = 0;
			}
			
			switch (seqState) {
				case 'W': {
					currentState = WAITSTATE;
					waitTime = seqValue;
					// if wait time is 0 use autoDelay (this allows for per match override from dashboard)
					if (waitTime == 0 ){
						waitTime = autoDelay;
					}
					timer.reset();
					timer.start();
					break;
				}
				case 'H':
				case 'D': {
					currentState = DRIVESTATE;
					waitTime = seqValue;
					if (seqState == 'H') {
						waitTime = waitTime/2;
					}
					timer.reset();
					timer.start();
					gyro.reset();  //reset current heading to 0 Degrees
					break;
			    }
				
				case 'L': { 
					currentState = LTURNSTATE;
				    aTurnAngle = -42 * seqValue; // auto turn angle is an increment of -45 degrees
				    gyro.reset();  //reset current heading to 0 Degrees
				    break;
				}
				case 'R': {
					currentState = RTURNSTATE;
					aTurnAngle = 42 * seqValue; //  auto turn angle is an increment of 45 degrees
					gyro.reset(); //reset current heading to 0 Degrees
					break; 
				}
				default:  // Default (or error) state is ENDSTATE
				case 'E': {
					currentState = ENDSTATE;
					timer.reset();
					break;
				}
			}
			
		}
	
}    // End all

/*
 SmartDashboard.putString("DB/String 1", autoSeq);
				SmartDashboard.putString("DB/String 2", gameData); 
				SmartDashboard.putString("DB/String 3", "");
				SmartDashboard.putString("DB/String 4", "");
	    	   	//SmartDashboard.putString("DB/String 5", "");
	    	   	SmartDashboard.putString("DB/String 6", Double.toString(Math.round(gyro.getAngle() * 1d)/1d ) + " Deg.");  // round to 0 decimal places
	    	   	SmartDashboard.putString("DB/String 7", "");
	    	   	SmartDashboard.putString("DB/String 8", Character.toString(seqState) + " State");
	            SmartDashboard.putString("DB/String 9", Double.toString(seqValue));  // 
 */
	