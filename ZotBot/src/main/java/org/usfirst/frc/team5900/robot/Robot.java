/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5900.robot;

//Tank Drive

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogOutput;


    


public class Robot extends IterativeRobot {
	private DifferentialDrive m_myRobot;
	private Joystick GameControl = new Joystick(0);
	private Joystick ControllerL = new Joystick(0);
	private Joystick ControllerR = new Joystick(1);
	
	
	double RobotMaxSpeed = .75;
	Compressor c = new Compressor(0);
	
	private SpeedController DriveMotorFront = new Spark(0);
	private SpeedController DriveMotorBack = new Spark (1);
	private SpeedController liftMotor = new Spark(2);
	private SpeedController Clamplift = new Spark (3);
	private Timer my_timer = new Timer();
	private AnalogOutput AngleFront = new AnalogOutput(0);
	private AnalogOutput AngleBack = new AnalogOutput(1);
	
	
	
	Solenoid MastLower = new Solenoid(0);
	Solenoid MastRaise = new Solenoid(1);
	
	Solenoid GripperClose = new Solenoid(2);
	Solenoid GripperOpen = new Solenoid(3);	   
	  
	
//	private ADXRS450_Gyro myGyro = new ADXRS450_Gyro();
	
	private int Maststate = 0;
	private int Gripperstate = 0;
	private int ATonOfMoose_State = 0;
	private int ATonOfMoose_Count = 0;
	private int Strategy_value = 0;
	private int AutoMove_State = 0;
	private int AutoMove_Count = 0;
	private int AutoMove_TargetCount = 0;
	private int Auto_Turn_State = 0;
	private int Auto_Turn_Count = 0;
	private int Auto_LiftMotor_State = 0;
	private int Auto_LiftMotor_Count = 0;
	private int Auto_ClampLift_State = 0;
	private int Auto_ClampLift_Count = 0;
	
	Preferences prefs;
	private boolean Center_Field;
	private boolean Left_Field;
	private boolean Right_Field;
	private boolean Switch_Target;
	private boolean Scale_Target;
	
	private boolean TurnRight;
	private boolean TurnLeft;
	
	
	double Kp = 0.03; 
	double angle;
		
	int TimeToSwitch = 1;
	double TurnAngle = 90.0;
	int TimeToTurn = 6;
	int TimeToBaseline = 8;
	int TimeToRight = 25;
	int TimeToDelta = 12;
	int TimeToLeft = 25;
	int TimeToLeft_Scale = 25;
	int TimeToRight_Scale = 25;
	int TimeToRight_Switch = 25;
	int TimeToLeft_Switch = 25;
	
	double TargetTime;
    double SpeedToSwitch;
    int TimeToScale = 5;
    int TimeToLiftMast = 2;
    double SpeedToMast = .3;
    double SpeedToLift = 0.5;
    double SpeedToClamp = 0.5;
    double VFront = 0.0;
    double VBack = 2.5;
        
    double leftspeed;
	double rightspeed;
	double rightturnangle;
	double leftturnangle;
	double frontangle;
	double backangle;
	
	boolean btnA;
	boolean btnB;
	boolean btnX;
	boolean btnY;
	boolean btnRightTrigger;
	boolean btnLeftTrigger;
	int POV_Angle;
	double lefttrigger;
	double righttrigger;
	boolean btnStart;
	boolean btnBack;
	
	double Deadband_Upper = 0.05;
	double Deadband_Lower = -0.05;
	
	
	DigitalInput limitSwitch = new DigitalInput(1);
	DigitalInput limitSwitch2 = new DigitalInput(2);
	DigitalInput limitSwitch3 = new DigitalInput(3);
	DigitalInput limitSwitch4 = new DigitalInput(4);
	DigitalOutput EnableFront = new DigitalOutput(5);
	DigitalOutput EnableBack = new DigitalOutput(6);
	DigitalOutput testOutput = new DigitalOutput(9);
	
	/***************************************************************************/
	/* autonmousInit() - Reading preferences and Value.                       */
	/*                                                                         */
	/***************************************************************************/
	@Override public void autonomousInit() 
	{
	/*
		Strategy_value = 0; 
		
		Center_Field = prefs.getBoolean("Center Field", false);
		Left_Field = prefs.getBoolean("Left Field", false);
		Right_Field = prefs.getBoolean("Right Field", false);
		Switch_Target = prefs.getBoolean("Switch Target", false);
		Scale_Target = prefs.getBoolean("Scale Target", false);
		
		TimeToSwitch = prefs.getInt("TimeToSwitch", 10);
		TurnAngle = prefs.getDouble("TurnAngle", 90.0);
		TimeToTurn = prefs.getInt("TimeToTurn", 50);
		TimeToBaseline = prefs.getInt("TimeToBaseline", 40);
	    SpeedToSwitch = prefs.getDouble("SpeedToSwitch", 0.4);
	    TimeToScale = prefs.getInt("TimeToScale", 13);
	    TimeToLiftMast = prefs.getInt("Time To Lift Mast", 20);
	    SpeedToMast = prefs.getDouble("Speed To Mast", 0.5); 
	    SpeedToLift = prefs.getDouble("Speed To Lift", 0.5);
	    SpeedToClamp = prefs.getDouble("Speed To Clamp", 0.5);
	    TimeToRight = prefs.getInt("Time to Right Edge of Switch", 25);
	    TimeToDelta = prefs.getInt("Time To Delta", 12);
	    TimeToLeft = prefs.getInt("Time To Left Edge of Switch", 25);
		TimeToLeft_Scale = prefs.getInt("Time To Left Scale", 25);
		TimeToRight_Scale = prefs.getInt("Time To Right Scale", 25);
	    TimeToLeft_Switch = prefs.getInt("Time To Left Switch", 25);
	    TimeToRight_Switch = prefs.getInt("Time To Right Switch", 25);
		
		SmartDashboard.putString("Mode", "Initializing AutoZot !!!!");
				
		ATonOfMoose_State = 0;
		my_timer.reset();
		my_timer.start();
		SmartDashboard.putString("Mode", "AutoZot Activated !!!!");
		*/
		
	}
	
	
	@Override public void autonomousPeriodic() 
	{
		//ATonOfMoose_Cntl();
		

	}
		       
	
	
	
	@Override public void robotInit() 
	{
		SmartDashboard.putString("Mode", "Robot Init" );
		m_myRobot = new DifferentialDrive(new Spark(5), new Spark(6));
		
		CameraServer.getInstance() .startAutomaticCapture(0);
	    c.setClosedLoopControl(true);
		
		prefs = Preferences.getInstance();
		Center_Field = prefs.getBoolean("Center Field", false);
		Left_Field = prefs.getBoolean("Left Field", false);
		Right_Field = prefs.getBoolean("Right Field", false);
		Switch_Target = prefs.getBoolean("Switch Target", false);
		Scale_Target = prefs.getBoolean("Scale Target", false);
		Auto_LowerMast();
	
	}
	 
	
	
	@Override public void teleopInit() 
	{
		SmartDashboard.putString("Mode", "Teleop Init" );
		Maststate = 0;
		Gripperstate = 0;	 
		TurnLeft = false;
		TurnRight = false;
		SmartDashboard.putBoolean("Right Turn", TurnRight);
		SmartDashboard.putBoolean("Left Turn", TurnLeft);
		EnableFront.set(true);	
	    EnableBack.set(true);	 
		
	}
	
	@Override public void disabledPeriodic() 
	{
	    EnableFront.set(false);	
	    EnableBack.set(false);	
	    AngleFront.setVoltage(0);
	    AngleBack.setVoltage(0);
	}
	
	
	@Override public void teleopPeriodic() 
	{
		
		/* Process Joysticks */
		leftspeed = GameControl.getRawAxis(1);
		leftturnangle = GameControl.getRawAxis(0);
		rightspeed = GameControl.getRawAxis(5);
		rightturnangle = GameControl.getRawAxis(4);
		leftspeed = GameControl.getY();
		
		

		/* Compensate for drift voltage of the Joysticks */
		if(( leftspeed < Deadband_Upper ) && ( leftspeed > Deadband_Lower ))
			leftspeed = 0.0;
		
		if(( leftturnangle < Deadband_Upper ) && ( leftturnangle> Deadband_Lower ))
			leftturnangle = 0.0;
		
		if(( rightspeed < Deadband_Upper ) && ( rightspeed > Deadband_Lower ))
			rightspeed = 0.0;
		
		if(( rightturnangle < Deadband_Upper ) && ( rightturnangle> Deadband_Lower ))
			rightturnangle = 0.0;
		
		SmartDashboard.putNumber("RightSpeed", rightspeed);
		SmartDashboard.putNumber("RightTurn", rightturnangle);
		SmartDashboard.putNumber("LeftSpeed", leftspeed);
		SmartDashboard.putNumber("LeftTurn", leftturnangle);
		
		/* Process Buttons */
		btnA = GameControl.getRawButton(1);
		btnB = GameControl.getRawButton(2);
		btnX = GameControl.getRawButton(3);
		btnY = GameControl.getRawButton(4); 	
		SmartDashboard.putBoolean("ButtonA", btnA);
		SmartDashboard.putBoolean("ButtonB", btnB);
		SmartDashboard.putBoolean("ButtonX", btnX); 
		SmartDashboard.putBoolean("ButtonY", btnY); 
		
		/* Process POV */
		POV_Angle = GameControl.getPOV();
		SmartDashboard.putNumber("POV Angle", POV_Angle);
		
		
		
		/* Process triggers + trigger buttons */
		lefttrigger = GameControl.getRawAxis(2);
		righttrigger = GameControl.getRawAxis(3);
		btnRightTrigger = GameControl.getRawButton(6);
		btnLeftTrigger = GameControl.getRawButton(5); 	
		SmartDashboard.putBoolean("BtnRightTrig", btnRightTrigger); 
		SmartDashboard.putBoolean("BtnLeftTrig", btnLeftTrigger);
		SmartDashboard.putNumber("LeftTrigger", lefttrigger);
		SmartDashboard.putNumber("RightTrigger", righttrigger);
		
		
		
		btnStart = GameControl.getRawButton(8);
		btnBack = GameControl.getRawButton(7); 	
		SmartDashboard.putBoolean("BtnStart", btnStart);
		SmartDashboard.putBoolean("BtnBack", btnBack);
		
		
		if( leftspeed != 0.0 )
		{
			// 0.0 V is -90 deg
			// 1.25V is -45 deg
			// 2.5 V is 0 deg
			// 3.75V is 45 deg
			// 5.0 V is 90 deg
			frontangle = (leftturnangle + 1)/2 * 5.0;  
			backangle = (leftturnangle + 1)/2 * 5.0; 
            AngleFront.setVoltage(frontangle);		
			AngleBack.setVoltage(backangle);
			DriveMotorFront.set(leftspeed);
			DriveMotorBack.set(leftspeed); 
			SmartDashboard.putString("Mode", "Translate");
			SmartDashboard.putNumber("Front Angle", frontangle);
			SmartDashboard.putNumber("Back Angle", backangle);
		}
		else if (rightspeed != 0.0 )    
		{
			if( rightturnangle > 0.5 )
			    rightturnangle = 0.5;    
			if( rightturnangle < -0.5 )
				rightturnangle = -0.5;
			
			frontangle = (rightturnangle + 1)/2 * 5.0;
			backangle = ( -rightturnangle + 1)/2 * 5.0;
			
			AngleFront.setVoltage(frontangle);		
			AngleBack.setVoltage(backangle);
			DriveMotorFront.set(rightspeed);
			DriveMotorBack.set(rightspeed);
			SmartDashboard.putString("Mode", "Arc");
			SmartDashboard.putNumber("Front Angle", frontangle);
			SmartDashboard.putNumber("Back Angle", backangle);
		}
		else
		{
			DriveMotorFront.set(0.0);
			DriveMotorBack.set(0.0);				
			SmartDashboard.putString("Mode", "Stopped");
			SmartDashboard.putNumber("Front Angle", frontangle);
			SmartDashboard.putNumber("Back Angle", backangle);
		}
		
		
		
		
		
	
		/*_myRobot.arcadeDrive(ControllerL.getY(), ControllerL.getX());
		
		leftspeed = ControllerR.getY();
		rightspeed = ControllerL.getY();
		
		
		if (leftspeed > RobotMaxSpeed )
			leftspeed = RobotMaxSpeed;
		else if (leftspeed < -RobotMaxSpeed )
		    leftspeed = -RobotMaxSpeed;
		
		if(rightspeed > RobotMaxSpeed )
			rightspeed = RobotMaxSpeed;
		else if (rightspeed < -RobotMaxSpeed )
			rightspeed = -RobotMaxSpeed;
		
	//	m_myRobot.tankDrive(leftspeed, rightspeed);
		
		if(ControllerL.getRawButton(3)) 
			liftMotor.set(-1.0);
		else 
		{	
		    if(ControllerL.getRawButton(2))
			    liftMotor.set(1.0);
		    else 
			    liftMotor.set(0.0);
		}
		
		if(ControllerR.getRawButton(3)) 
			Clamplift.set(-0.5);			
		else
		{	
			if(ControllerR.getRawButton(2)) 
		  	    Clamplift.set(0.5);				
		    else 
			    Clamplift.set(0.0);
		}
		
		if (prefs.getBoolean("TestSwith", false) == true)
			testOutput.set(true);
		else
			testOutput.set(false);
		
		Mast_Cntl();
		Gripper_Cntl();
	*/
		
    /*
		if (limitSwitch.get() == false && limitSwitch2.get() == true )
		{
			Clamplift.set(0.75); 
			DriveMotorA.set(-0.75);
		    SmartDashboard.putString("Mode", "Turn Right");
		}
		else if (limitSwitch2.get() == false && limitSwitch.get() == true)
		{
			Clamplift.set(-0.75);
			DriveMotorA.set(0.75);
			SmartDashboard.putString("Mode", "Turn On Reverse");
		}
		else if (limitSwitch2.get() == false && limitSwitch.get() == false)
		{
			Clamplift.set(0.75);
			DriveMotorA.set(0.75);
			SmartDashboard.putString("Mode", "Turn On Reverse");
		}
		else
		{
			Clamplift.set(0.0);
			DriveMotorA.set(0.0);
			SmartDashboard.putString("Mode", "Turn Off");
		}
		
		
		if (limitSwitch3.get() == false )
		{
			liftMotor.set(0.75);
			TurnMotorA.set(0.75);
			SmartDashboard.putString("Mode", "Turn On Forward");
		}
		else if (limitSwitch4.get() == false )
		{
			liftMotor.set(-0.75);
			TurnMotorA.set(-0.75);
			SmartDashboard.putString("Mode", "Turn On Reverse");
		}
		else 
		{
			liftMotor.set(0.0);
			TurnMotorA.set(0.0);
			SmartDashboard.putString("Mode", "Turn Off");
		}
		
		
		AngleFront.setVoltage(leftturnangle * 5.0);
		if (VFrontDir)
		{ 
		  VFront += 0.1;
		  if (VFront > 5.0)
		  {
		    VFront = 5.0;
		    VFrontDir = false;
		  }
		}
		else
		{
			VFront -= 0.1;
			  if (VFront < 0.0)
			  {
			    VFront = 0.0;
			    VFrontDir = true;
			  }	
		}
		
		
		AngleBack.setVoltage((rightturnangle + 1)/2 * 5.0);
		VBack += 0.1;
		if (VBack > 5.0)
		  VBack = 0;
		
		Center_Field = prefs.getBoolean("Center Field", false);
		Left_Field = prefs.getBoolean("Left Field", false);
		Right_Field = prefs.getBoolean("Right Field", false);
		
		if (Center_Field == true)
			SmartDashboard.putString("Direction", "Straight");
		else if (Left_Field == true)
			SmartDashboard.putString("Direction", "Left");
		else if (Right_Field == true)
			SmartDashboard.putString("Direction", "Right");   
	*/
		
	}
	
	
	/* This controls the raise and lower the mast.           */
	/* Pulling trigger will raise the mast.                  */ 
	/* Release the trigger and pull the trigger again lowers */
	/* the mast.                                             */
	public void Mast_Cntl() 
	{   
		switch (Maststate) 
		{
		case 0://waiting for trigger press
			if (ControllerL.getRawButton(1) == true) 
			{ 
			    MastRaise.set(true); 
			    MastLower.set(false);
			    Maststate = 1;
			}
			break;
			
		case 1: //waiting for trigger release
			if (ControllerL.getRawButton(1) == false) 
			    Maststate = 2;
			break;
			
		case 2: 
			if (ControllerL.getRawButton(1) == true) 
			{ 
				MastRaise.set(false);
				MastLower.set(true);
				Maststate = 3;
			}
			break;
			
		case 3: //waiting for trigger release
			if (ControllerL.getRawButton(1) == false)
			    Maststate = 0;
			break; 
			
		default: 
			Maststate = 0;
			MastRaise.set(false);
			MastLower.set(false); 
			break;

		}
	}
	
	/* This controls the opening and closing of the gripper  */
	/* Pulling trigger will close the gripper                */
	/* Release the trigger and pull the trigger again opens  */
	/* the gripper.                                          */
	public void Gripper_Cntl() 
	{
		switch (Gripperstate) 
		{
		case 0://waiting for trigger press
			if (ControllerR.getRawButton(1) == true) 
			{ 
			    Auto_GripperClose();
			    Gripperstate = 1;
			}
			break;
			
		case 1: //waiting for trigger release
			if (ControllerR.getRawButton(1) == false) 
			    Gripperstate = 2;
			break;
			
		case 2: 
			if (ControllerR.getRawButton(1) == true) 
			{ 
				Auto_GripperOpen();
				Gripperstate = 3;
			}
			break;
			
		case 3: //waiting for trigger release
			if (ControllerR.getRawButton(1) == false)
			    Gripperstate = 0;
			break;
			
		default: 
			Gripperstate = 0;
			GripperClose.set(false);
			GripperOpen.set(false);
			break;

		}
	}
	
	
	
	/***************************************************************************/
	/* ATonOfMoose_Cntl() - autonomous state controller.                       */
	/*                                                                         */
	/***************************************************************************/
	public void ATonOfMoose_Cntl()
	{
		switch (ATonOfMoose_State)
		{
		case 0:
			my_timer.reset(); 
	    	my_timer.start();
	        ATonOfMoose_State = 1;
	        ATonOfMoose_Count = 0;  
	    	//myGyro.reset();
	    	Auto_RaiseMast();    	
	    	SmartDashboard.putString("Mode", "Waiting for Game Data");
			break;
			     
			
		case 1:  // Time delay to insure fresh game data
			if ( ++ATonOfMoose_Count > 75 )                 // 75 * 20 msec 
			{
				CalcStrategy();
				switch(Strategy_value)
				{
				case 0:
				case 1:
					SmartDashboard.putString("Mode", "Move to Right Switch, Place ");
					AutoMove_State = 0;
					Auto_Turn_State = 0;
					ATonOfMoose_State = 100;   
					break;
					
				case 2:
				case 3:
					SmartDashboard.putString("Mode", "Move to Left Switch, Place ");
					AutoMove_State = 0;
					Auto_Turn_State = 0;
					ATonOfMoose_State = 200;   
					break;
					
				case 7:
				case 8:
					SmartDashboard.putString("Mode", "Move Line ");
					AutoMove_State = 0;
					Auto_Turn_State = 0;
					ATonOfMoose_State = 300;   
					break;
				
				case 4:
				case 6:
					SmartDashboard.putString("Mode", "Move to Left Scale Line, Place ");
					AutoMove_State = 0;
					Auto_Turn_State = 0;
					ATonOfMoose_State = 400;  
					break;
				
				case 5:
					SmartDashboard.putString("Mode", "Move to Left Switch, Place ");
					AutoMove_State = 0;
					Auto_Turn_State = 0;
					ATonOfMoose_State = 500;  // "Me to Switch, L, Place"
					break;
				
				case 9:
				case 11:
					SmartDashboard.putString("Mode", "Move to Right Scale, Place ");
					AutoMove_State = 0;
					Auto_Turn_State = 0;
					ATonOfMoose_State = 600;  
					break;
				
				case 10:
					SmartDashboard.putString("Mode", "Move to Right Switch, Place ");
					AutoMove_State = 0;
					Auto_Turn_State = 0;
					ATonOfMoose_State = 700;  
					break;
				
				default:
					SmartDashboard.putString("Mode", "Move to Line ");
					AutoMove_State = 0;
					Auto_Turn_State = 0;
					ATonOfMoose_State = 100;   
					break;
				}		
			}
			else
				CalcStrategy();
			break;
			
		case 100: //Center place on right side of switch
			ATonOfMoose_State = 110;
			AutoMove_State = 0;
			SmartDashboard.putString("State", "110 Move to base line" );
			break;
			
		case 110:
			Auto_Move( TimeToBaseline );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 120;
				SmartDashboard.putString("State", "120 Turning 90deg" );
			}
			break;
			
		case 120:
			Auto_Turn (90);
			if(Auto_Turn_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 130;
				AutoMove_State = 0;
				SmartDashboard.putString("State", "130 Move to Right" );			
			}
			break;
			
		case 130:
			Auto_Move( TimeToRight );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 140;
				SmartDashboard.putString("State", "140 Turning -90deg" );
			}
			break;
			
		case 140:
			Auto_Turn(-90);
			if(Auto_Turn_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 150;
				AutoMove_State = 0;
				SmartDashboard.putString("State", "150 Move TimeToDelta" );
			}
			break;
			
		case 150:
			Auto_Move( TimeToDelta );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 1000;
				SmartDashboard.putString("State", "1000 Complete - Open Gripper" );
				Auto_GripperOpen();
			}
			break;
			
		case 200: //Center place on left side of switch
			ATonOfMoose_State = 210;
			AutoMove_State = 0;
			SmartDashboard.putString("State", "210 Move to Baseline" );
			break;
			
		case 210:
			Auto_Move( TimeToBaseline );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 220;
				SmartDashboard.putString("State", "220 Turn -90deg" );
			}
			break;
			
		case 220:
			Auto_Turn (-90);
			if(Auto_Turn_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 230;
				AutoMove_State = 0;
				SmartDashboard.putString("State", "230 Move TimeToLeft" );
				
			}
			break;
			
		case 230:
			Auto_Move( TimeToLeft );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 240;
				SmartDashboard.putString("State", "240 Turn 90deg" );
			}
			break;
			
		case 240:
			Auto_Turn(90);
			if(Auto_Turn_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 250;
				AutoMove_State = 0;
				SmartDashboard.putString("State", "250 Move TimeToDelta" );
			}
			break;
			
		case 250:
			Auto_Move( TimeToDelta );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 1000;
				Auto_GripperOpen();
				SmartDashboard.putString("State", "1000 Complete Open Gripper" );
			}
			break;
			
		case 300: //Right or Left Field Move To Line
			ATonOfMoose_State = 310;
			AutoMove_State = 0;
			SmartDashboard.putString("State", "310 Move to Baseline" );
			break;
			
		case 310:
			Auto_Move( TimeToBaseline );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 1000;
				SmartDashboard.putString("State", "1000 Move to Baseline" );			
			}
			break;
			
		case 400:
			ATonOfMoose_State = 410;
			AutoMove_State = 0;
			SmartDashboard.putString("State", "410 Move to TimeToScale" );
			break;
			
		case 410:
			Auto_Move( TimeToScale );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 420;
				SmartDashboard.putString("State", "420 Turn -90deg" );
			}
			break;
			
		case 420:
			Auto_Turn (-90);
			if(Auto_Turn_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 430;
				AutoMove_State = 0;
				SmartDashboard.putString("State", "430 Move to TimeToRight_Scale" );
			}
			break;
			
		case 430:
			Auto_Move( TimeToRight_Scale );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 1000;
				Auto_GripperOpen();
				SmartDashboard.putString("State", "430 Complete - Open Gripper" );
			}
			break;
			
		case 500: //Right Field, Go To Switch, Turn Left
			ATonOfMoose_State = 510;
			AutoMove_State = 0;
			SmartDashboard.putString("State", "510 Move to TimeToSwitch" );
			break;
			
		case 510:
			Auto_Move( TimeToSwitch );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 520;
				SmartDashboard.putString("State", "520 Turn -90deg" );
			}
			break;
			
		case 520:
			Auto_Turn (-90);
			if(Auto_Turn_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 530;
				AutoMove_State = 0;
				SmartDashboard.putString("State", "530 Move to TimeToRight_Switch" );
			}
			break;
			
		case 530:
			Auto_Move( TimeToRight_Switch );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 1000;
				Auto_GripperOpen();
				SmartDashboard.putString("State", "1000 Complete - Open Gripper" );
			}
			break;
		
		case 600:
			ATonOfMoose_State = 610;
			AutoMove_State = 0;
			SmartDashboard.putString("State", "610 Move to TimetoScale" );
			break;
			
		case 610:
			Auto_Move( TimeToScale );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 620;
				SmartDashboard.putString("State", "620 Turn 90deg" );
			}
			break;
			
		case 620:
			Auto_Turn (90);
			if(Auto_Turn_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 630;
				AutoMove_State = 0;
				SmartDashboard.putString("State", "620 Move to TimeToLeft_Scale" );
			}
			break;
			
		case 630:
			Auto_Move( TimeToLeft_Scale );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 1000;
				Auto_GripperOpen();
				SmartDashboard.putString("State", "1000 Complete - Gripper Open" );
			}
			break;
		
		case 700: //Left Field, Go To Switch, Turn Right
			ATonOfMoose_State = 710;
			AutoMove_State = 0;
			SmartDashboard.putString("State", "710 Move to TimeToSwitch" );
			break;
			
		case 710:
			Auto_Move( TimeToSwitch );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 720;
				SmartDashboard.putString("State", "720 Turn 90deg" );
			}
			break;
			
		case 720:
			Auto_Turn (90);
			if(Auto_Turn_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 730;
				AutoMove_State = 0;
				SmartDashboard.putString("State", "730 Move to TimeToLeft_Switch" );
			}
			break;
			
		case 730:
			Auto_Move( TimeToLeft_Switch );
			if(AutoMove_State == 100)
			{
				Auto_Turn_State = 0;
				ATonOfMoose_State = 1000;
				Auto_GripperOpen();
				SmartDashboard.putString("State", "1000 Complete - Gripper Open" );
			}
			break;
			
		case 1000:
			m_myRobot.arcadeDrive(0.0, 0.0);
			break;
			
    	default:
    		m_myRobot.arcadeDrive(0.0, 0.0);
    		ATonOfMoose_State = 0;
    		break;
    	
		}
	}
	
	
	/***************************************************************************/
	/* Auto_Move() - automatically move forward for period of time.            */
	/*               TargetCount is in 20ms units                              */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_Move( int TargetCount )
	{
		SmartDashboard.putNumber("AutoMoveState", AutoMove_State);
		switch (AutoMove_State)
		{
		case 0:
			SmartDashboard.putString("Mode", "moving forward");
			AutoMove_Count = 0;
			AutoMove_TargetCount = TargetCount;  // time = TargetCount * 20ms
			AutoMove_State = 1;
			break;
			
		case 1:
			if ( ++AutoMove_Count > AutoMove_TargetCount )
    		{ 
    //		    angle = myGyro.getAngle(); // get current heading
    		    m_myRobot.arcadeDrive(-0.35, -angle*Kp); // drive towards heading 0	    		   	    		        	
                SmartDashboard.putString("Mode", "slowing down");
                AutoMove_Count = 0;
                AutoMove_TargetCount = 5;
    		    AutoMove_State = 2;
    		}
    		else 
    		{ 
 	//	        angle = myGyro.getAngle(); // get current heading
                m_myRobot.arcadeDrive(-0.6, -angle*Kp); // drive towards heading 0	                     	
    		} 
			break;
			
		case 2:
			if ( ++AutoMove_Count > AutoMove_TargetCount )
    		{
                m_myRobot.arcadeDrive(0.0, 0.0); 
                SmartDashboard.putString("Mode", "stopped");
                AutoMove_State = 100;                			
    		}   
    		else 
    		{
    //			angle = myGyro.getAngle(); // get current heading
                m_myRobot.arcadeDrive(-0.25, -angle*Kp); // drive towards heading 0	                  		
    		}
			break;
			
		case 100:  // Move complete
			break;
			
		default:
			m_myRobot.arcadeDrive(0.0, 0.0); 
			SmartDashboard.putString("Mode", "stopped");
            AutoMove_State = 100;                
			break;			
			
		}
		
	}
	/***************************************************************************/
	/* Auto_Turn() - Automatically turn a specific angle.					   */
	/* 				 Positive Angle Clockwise			                       */
	/*				 Negative Angle Counter Clockwise						   */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_Turn(double Target_angle)
	{
		
		switch (Auto_Turn_State)
		{
		case 0:
			if (Target_angle >= 0 )
            {
			    Auto_Turn_Count = 0;
             	Auto_Turn_State = 1;             
            }
            else 
            {
            	Auto_Turn_Count = 0;
             	Auto_Turn_State = 10;
            }
		    break;
			
		case 1:
			m_myRobot.tankDrive(0.5, -0.5);
 	//		angle = myGyro.getAngle(); // get current heading
         	if (angle > Target_angle )
         	{
         		SmartDashboard.putString("Mode", "turn complete");
         		m_myRobot.arcadeDrive(0.0, 0.0);
         		Auto_Turn_Count = 0;
         		Auto_Turn_State = 2;	 	                	
         	}
         	else 
         	{
         		if( ++Auto_Turn_Count > 2500 )
         		{
         			SmartDashboard.putString("Mode", "stopped turning error");
         			m_myRobot.arcadeDrive(0.0, 0.0);
             		Auto_Turn_State = 100;
         		}
         	}
			break;
				
		case 2:
			if( ++Auto_Turn_Count > 20 )
     		{
     			SmartDashboard.putString("Mode", "stopped turning error");
     			m_myRobot.arcadeDrive(0.0, 0.0);
         		Auto_Turn_State = 3;
     		}
			else
				m_myRobot.arcadeDrive(0.0, 0.0);
			break;
			
		case 3:
//			myGyro.reset();
			Auto_Turn_Count = 0;
			Auto_Turn_State = 4;
			m_myRobot.arcadeDrive(0.0, 0.0);
			break;
			
		case 4:
			if( ++Auto_Turn_Count > 10 )
     		{
     			SmartDashboard.putString("Mode", "Reset Gyro");
     			m_myRobot.arcadeDrive(0.0, 0.0);
         		Auto_Turn_State = 100;
     		}
			break;
			
		case 10:
			m_myRobot.tankDrive(-0.5, 0.5);
 //			angle = myGyro.getAngle(); // get current heading
         	if (angle < Target_angle )
         	{
         		SmartDashboard.putString("Mode", "turn complete");
         		m_myRobot.arcadeDrive(0.0, 0.0);
         		Auto_Turn_State = 2;	 	                	
         	}
         	else 
         	{
         		if( ++Auto_Turn_Count > 2500 )
         		{
         			SmartDashboard.putString("Mode", "stopped turning error");
         			m_myRobot.arcadeDrive(0.0, 0.0);
             		Auto_Turn_State = 100;
         		}
         	}
         	break;
			
		case 100:
			m_myRobot.arcadeDrive(0.0, 0.0);
			break;
			
		default:
			Auto_Turn_State = 100;
			m_myRobot.arcadeDrive(0.0, 0.0); 
			SmartDashboard.putString("Mode", "stopped in unknown state");
			break;
		}
	}

	/***************************************************************************/
	/* Auto_LiftMotor() - automatically Lifts up the mass.                       */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_LiftMotor(int Target_Count)
	{
		
		switch (Auto_LiftMotor_State)
		{
		case 0:
			Auto_LiftMotor_Count = 0;
			Auto_LiftMotor_State = 1;
			break; 
			
		case 1:
			if ( ++Auto_LiftMotor_Count > Target_Count )
			{
				liftMotor.set(0.0);
				Auto_LiftMotor_State = 100;
			}
			else
			{
				liftMotor.set(SpeedToLift);
			}
			break;
			
		case 2:
			if ( ++Auto_LiftMotor_Count > Target_Count )
			{
				liftMotor.set(0.0);
				Auto_LiftMotor_State = 100;
			}
			else
			{
				liftMotor.set(-SpeedToLift);
			}
			break;
			
		case 100:
			liftMotor.set(0.0);
			break;
			
		default:
			Auto_LiftMotor_State = 100;
			break;
		}
	}
	
	/***************************************************************************/
	/* Auto_LiftMotor() - automatically Lifts up the mass.                       */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_ClampLift(int Target_Count)
	{
		
		switch (Auto_ClampLift_State)
		{
		case 0:
			Auto_ClampLift_Count = 0;
			Auto_ClampLift_State = 1;
			break; 
			
		case 1:
			if ( ++Auto_ClampLift_Count > Target_Count )
			{
				Clamplift.set(0.0);
				Auto_ClampLift_State = 100;
			}
			else
			{
				Clamplift.set(SpeedToClamp);
			}
			break;
			
		case 2:
			if ( ++Auto_ClampLift_Count > Target_Count )
			{
				Clamplift.set(0.0);
				Auto_ClampLift_State = 100;
			}
			else
			{
				Clamplift.set(-SpeedToClamp);
			}
			break;
			
		case 100:
			Clamplift.set(0.0);
			break;
			
		default:
			Auto_ClampLift_State = 100;
			break;
		}
	}
	
	/***************************************************************************/
	/* Auto_GripperOpen() - automatically opens gripper.                       */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_GripperOpen()
	{
		GripperClose.set(false);
		GripperOpen.set(true);	
	}
	
	
	/***************************************************************************/
	/* Auto_GripperClose() - automatically opens gripper.                       */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_GripperClose()
	{
		GripperClose.set(true);
		GripperOpen.set(false);	
	}
	
	
	/***************************************************************************/
	/* Auto_RaiseMast() - automatically raise mast.                            */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_RaiseMast()
	{
		MastRaise.set(true);
		MastLower.set(false);	
	}

	
	/***************************************************************************/
	/* Auto_LowerMast() - automatically lowers mast.                           */
	/*                                                                         */
	/***************************************************************************/
	public void Auto_LowerMast()
	{
		MastRaise.set(false);
		MastLower.set(true);	
	}
	
		
	/***************************************************************************/
	/* CalcStrategy() - given field location and game data, calculate a        */
	/*                  strategy value.                                        */
	/*                                                                         */
	/***************************************************************************/
	public void CalcStrategy() 
	{
		String gameData;
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		Strategy_value = 0;
		
		if( Center_Field ) 
		{
			SmartDashboard.putString("Field", "Center Field" );
			Strategy_value = 0;
		}
		else if( Right_Field )
		{
			SmartDashboard.putString("Field", "Right Field" );
			Strategy_value = 4;
		}
		else
		{
			SmartDashboard.putString("Field", "Left Field" );
			Strategy_value = 8;
		}
		
		
		if(gameData.length() > 0)
        {
			SmartDashboard.putString("Game Data", gameData );
        	
        	if(gameData.charAt(0) == 'L')
                Strategy_value = Strategy_value + 2;
            
		    if(gameData.charAt(1) == 'L')		    
		       	Strategy_value = Strategy_value + 1;		          
        }
		
		SmartDashboard.putNumber("Strategy Value", Strategy_value);
	
	}	
}




