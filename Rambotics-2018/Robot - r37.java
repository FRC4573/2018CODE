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
	boolean mapBuilt; // 
	
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
	char charTmp;
	int seqValue;
	int seqIndex;
	int retries;//Gamedata vaild retries
	
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
		m_elevatorSpeed = 1.00;
		displayCtr = 0;
		seqIndex = -1;
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
		autoSeq = "W0D7W9W5E0";
		autoMap = "";
		retries = 1;
		SmartDashboard.putString("DB/String 5", "R");
		SmartDashboard.putString("DB/String 6", "0");
		SmartDashboard.putString("DB/String 7", "1");
		updateDisplays();
		
	}

	 //******************************************************************
	 // this routine is called 1 time at the beginning of autonomous mode
	 //******************************************************************
	
	@Override
	public void autonomousInit() {
		System.out.println("AutoInit"); 
		mapBuilt = false;
		retries = 50; // wait up to 2 secs for game data
		gyro.reset();  // Reset Gyro at start of Autonomous
		seqIndex = -1;
		autoMap = "";
		autoProg = 0;
		autoDelay = 0;
/*				
   //temp overide	
	autoSeq = "W0D4W9W5E0"; // If no game data, just break the autoline
		getNextState();
		mapBuilt = true; 
	
	//end temp
		mapBuilt = true; */
//		updateDisplays();
		
		// at start of auto close grabber and extend drawer
		closeGrabber();
		extendDrawer();
		System.out.println("EndAutoInit"); 
	}
	
	 //********************************************************************
	 // this routine is called every 20 milliseconds during autonomous mode
	 //********************************************************************
	
	@Override
	public void autonomousPeriodic() {
		
		updateDisplays();	
	// In auto Periodic, wait for valid game data from FMS before initializing for auto, otherwise just break autoline if timeout occurs
		if (!mapBuilt) {
			m_drive.arcadeDrive(0, 0); // stop driving
			gameData = DriverStation.getInstance().getGameSpecificMessage(); //Read game data from FMS
			if (gameData.length()>2 ) {
				autoInit(); // get game data if valid 
				getNextState();
			}
			else {
				retries--;
				if (retries < 1){
					autoSeq = "W0D7W9W5E0"; // If no game data, just break the autoline
					getNextState();   
					mapBuilt = true;
					System.out.println("NoGameData");
				}
				
			}
		}  
		// raise elevator in auto mode (while running through states/approaching the switch
		if(limitSwUpper.get()==INACTIVE) {
			m_elevator.set(-m_elevatorSpeed); // raise elevator while limit switch is inactive
		}
		else {
			m_elevator.set(0.0); // Stop elevator
		}

		if (mapBuilt){ // only if Map is built, then cycle through states
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
						m_drive.arcadeDrive(m_autoDriveSpeed, -gyro.getAngle()*.1); //was .08
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
					openGrabber(); // drop the cube (hopefully onto the switch)
					m_drive.arcadeDrive(0, 0); // stop driving
					break;
				}
			
			}
		}
		
	}

	 //***************************************************************
	 // this routine is called 1 time at the beginning of teleop mode
	 //***************************************************************
		
	@Override
	public void teleopInit() {
		System.out.println("TeleOpInit"); 
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
          Y=0; // set speed to 0 if elevator tries to move past limit switches
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
				m_elevator.set(1.0); // when climbing, Lower elevator while limit switch is inactive
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
				displayCtr = 0; // reset display CTR
				SmartDashboard.putString("DB/String 0", "StartingPos(L,C,R)");
				SmartDashboard.putString("DB/String 1", "StartDelay(secs)");
				SmartDashboard.putString("DB/String 2", "AutoProg(0 - 1)");
				SmartDashboard.putString("DB/String 3", autoSeq);
				SmartDashboard.putString("DB/String 4", gameData); 
	    	   	//SmartDashboard.putString("DB/String 5", "");
	    	   	//SmartDashboard.putString("DB/String 6","");  
	    	    //SmartDashboard.putString("DB/String 7", "");
	    	 	SmartDashboard.putString("DB/String 8", Integer.toString(retries));  // round to 0 decimal places
	            SmartDashboard.putString("DB/String 9", autoMap);  // 
	           // SmartDashboard.putString("DB/String 8", Character.toString(seqState) + " State");
	           // SmartDashboard.putString("DB/String 9", Double.toString(seqValue));  // 
		 	}
			displayCtr++;
		}
		// ***************************************************************************
	    // This function parses the auto sequence string, and
		// Set variables up for the corresponding state.
		// The string can be any sequence of 2 character commands as follows:
		// Format of AutoCommands:
		// Wn = Wait n seconds ; IF n=0 the dashboard autoDelay over-ride value is used
		// Dn = Drive forward for n seconds: 5secs = 11ft
		// Lm = Turn Left m * 45 degrees 
		// Rm = Turn Right m * 45 degrees
		// E0 = End State - release (open) Grabber
		// where n and m must be between 0 and 9 inclusive
		//
		// Example: W0D2R2W3E0 = Wait 0 (use AutoDelay,  Drive Fwd 2 secs, Turn Right 90 Deg, Wait 3 secs, End 
	    // ***************************************************************************
		
		public void getNextState() 	{
			 
			seqIndex++; //increment before use
			seqState = autoSeq.charAt(seqIndex); // read sequence command
			seqIndex++;
			seqValue = Character.getNumericValue(autoSeq.charAt(seqIndex)); // read sequence value as an integer
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
	
		// ***************************************************************************
		// AUTO INIT function: read values from dashboard for match specific settings
		// It seems the dashboard defaults don't work when using FMS at competition:
		// You MUST actually type in dashboard values at match setup - otherwise null
		// values are returned!!!
		// ***************************************************************************
		public void autoInit() 	{
		
			autoTemp = SmartDashboard.getString("DB/String 6", "0");  
			if (autoTemp==null || autoTemp.equals("")){
				autoTemp = "0"; // Default = 0 seconds if none entered
				System.out.println("AutoInit - Null-1");
			}
			autoDelay = Character.getNumericValue(autoTemp.charAt(0)); // Set autoDelay value per dashboard
			
			autoTemp = SmartDashboard.getString("DB/String 7", "1");  
			if (autoTemp == null || autoTemp.equals("")){
				autoTemp ="1"; // Default = program 1 if none selected
				System.out.println("AutoInit - Null-2"); 
			}
			autoProg = Character.getNumericValue(autoTemp.charAt(0)); // Set autoProgram per dashboard
			
			autoMap = SmartDashboard.getString("DB/String 5", "R");  
			if (autoMap==null || autoMap.equals("")){
				autoMap ="R"; // Default = Right if none selected
				System.out.println("AutoInit - Null-3"); 
			}
			autoMap += Character.toString(gameData.charAt(0)); // set autoMap to starting position & Gamedata switch ownership 
		 
		switch(autoMap) {
			case "LL":{
				autoSeq = "W0D1R1D4L1D1E0";// Hit from side of switch
				if (autoProg == 1) {
					autoSeq = "W0D7R2D2E0";  // Tested - Drop on front tip of the switch
				}
				if (autoProg == 2) {
					autoSeq = "W0D7W9W5E0";// For Prog 2 - Just break Autoline
				}
				break;
			}
			case "LR":{
				
				autoSeq = "W0D7W9W5E0"; //JUST break autoline
				break;
			}
			case "CL":{
				autoSeq = "W0D1L1D4R1D1E0"; // Tested
				break;
			}
			case "CR":{
				autoSeq = "W0D1R1D3L1D2E0"; // Tested
				break;
			}
			case "RL":{ 
				autoSeq = "W0D7W9W5E0";//  Just break the autoline and wait
				break;
			}
			case "RR":{
				autoSeq = "W0D1L1D4R1D1E0";// Tested - Hit from side of switch
				if (autoProg == 1) {
					autoSeq = "W0D7L2D2E0";  // Tested - Drop on front tip of the switch
				}
				if (autoProg == 2) {
					autoSeq = "W0D7W9W5E0";// For Prog 2 - Just break Autoline
				}
				break;
			}
			default:{
				autoSeq = "W0D7W9W5E0"; // If unknown automap, just break the autoline
			}
		
		} 
			
		mapBuilt = true;	
	}

}    // End all

