ackage frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.networktables.*;

import java.math.*;
import java.util.*;


public class Robot extends SampleRobot {
  TalonSRX LeftDrive = new TalonSRX(20);

  VictorSPX LeftDrive2 = new VictorSPX(21);
 
  TalonSRX RightDrive = new TalonSRX(22);
  VictorSPX RightDrive2 = new VictorSPX(23);

  
  VictorSP IntakeRight = new VictorSP(0);
  VictorSP IntakeLeft = new VictorSP(1);

  TalonSRX Lift1 = new TalonSRX(24);
  TalonSRX Lift2 = new TalonSRX(25);
  Joystick Xbox = new Joystick(0);
 
 
  // NetworkTable objects
  NetworkTableEntry x;
  NetworkTableEntry y;
  NetworkTableEntry hl;
  NetworkTableEntry hu;
  NetworkTableEntry sl;
  NetworkTableEntry su;
  NetworkTableEntry vl;
  NetworkTableEntry vu;
  NetworkTableEntry radius;
  public Robot() {
    
  }

  @Override
  public void robotInit() {
    //LeftDrive.configFactoryDefault();
    LeftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    //RightDrive.configFactoryDefault();
    RightDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //Set nominal and peek voltage of the drive train motors
    LeftDrive.configNominalOutputForward(0);
    LeftDrive.configNominalOutputReverse(0);
    LeftDrive.configPeakOutputForward(100);
    LeftDrive.configPeakOutputForward(100);
    LeftDrive.configPeakCurrentLimit(32, 50);
    LeftDrive.configPeakCurrentDuration(75, 20);

    RightDrive.configNominalOutputForward(0);
    RightDrive.configNominalOutputReverse(0);
    RightDrive.configPeakOutputForward(100);
    RightDrive.configPeakOutputForward(100);
    RightDrive.configPeakCurrentLimit(32, 50);
    RightDrive.configPeakCurrentDuration(75, 20);

    //Values to tune PID Loops on talons for velocity control
    LeftDrive.selectProfileSlot(0, 0);
    LeftDrive.config_kF(0,0,20);
    LeftDrive.config_kP(0, 0, 20);
    LeftDrive.config_kI(0,0,20);
    LeftDrive.config_kD(0, 0, 20);
    
    RightDrive.selectProfileSlot(0, 0);
    RightDrive.config_kF(0,0,20);
    RightDrive.config_kP(0, 0, 20);
    RightDrive.config_kI(0,0,20);
    RightDrive.config_kD(0, 0, 20);
    //if your drive train has 3 motor controllers per side just add another follower
    LeftDrive2.follow(LeftDrive);
    RightDrive2.follow(RightDrive);

  // Set up and populate the networkTable
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("SmartDashboard");
  x = table.getEntry("X");
  y = table.getEntry("Y");
  radius = table.getEntry("R");
  hl = table.getEntry("HL");
  hu = table.getEntry("HU");
  sl = table.getEntry("SL");
  su = table.getEntry("SU");
  vl = table.getEntry("VL");
  vu = table.getEntry("VU");

  //Default values, you will need to changes these when you finish tuning
  hl.setDefaultDouble(0);
  hu.setDefaultDouble(0);
  sl.setDefaultDouble(0);
  su.setDefaultDouble(0);
  vl.setDefaultDouble(0);
  vu.setDefaultDouble(0);
   }
   
   
  
   @Override
  
  public void autonomous() {
      /*
      * This is a P Controller where the target is always 0
      * the P normally will settle when the ball is about 1 foot away.
      * To change The Distance the Robot stops, increse or decrease NeutralOffSet.
      * To affect how much the robot wants to drive towards or away from the ball
      * Increase PGain
      * 
      *The other P controller Figures out how far left or right to turn the robot
      *Works Roughly the same way, X represcents the x cordnate of the center 
      *of the targe
      */

      //Varibles for Radius P controller
      double MaxRadius = 60;
      double MinRadius = 10;
      double MaxOutR = 1;
      double MinOutR = -1;
      double NeutralOffSetR= 0.25;
      double PGainR = 0.250;

      //Varibles for X P controller
      double MaxX = 160;
      double MinX = 10;
      double Maxx = 1;
      double Minx= -1;
      double NeutralOffSetX= 0;
      double PGainX = 0.25;

      //The Varibles sent to the TalonSRX's
      double ScaledX = 0;
      double ScaledY = 0;
      double ScaledRadius = 0;
      LeftDrive2.follow(LeftDrive);
      RightDrive2.follow(RightDrive);
      while(isEnabled()){

        double X = Math.round(x.getDouble(-1));
        double Y = Math.round(y.getDouble(-1));
        double Radius =  radius.getDouble(-1);
        if(X == -1){
         ScaledX = 0;
         ScaledY = 0;
         ScaledRadius = 0;
        } else {
        ScaledX =  PGainX * ((((Maxx - Minx)*((X- MinX)/(MaxX - MinX))) + Minx) -  NeutralOffSetX);
        ScaledRadius =  PGainR * ((((MaxOutR - MinOutR)*((Radius - MinRadius)/(MaxRadius - MinRadius))) + MinOutR) -  NeutralOffSetR);
        }
        
        Double LeftSpeed = -(ScaledRadius +  ScaledX) - 0.03;
        Double RightSpeed = ScaledRadius -  ScaledX;
        //System.out.println("ScaledX: "+ ScaledX + " ScaledRadius: " + ScaledRadius);
        System.out.println("LeftSpeed: "+ LeftSpeed + " RightSpeed: " + RightSpeed);
        LeftDrive.set(ControlMode.PercentOutput,  -RightSpeed);
        RightDrive.set(ControlMode.PercentOutput, -LeftSpeed);

        Timer.delay(0.01);
      }
    }
  

  
  @Override
  public void operatorControl() {
    //Delcare Varibles and lists
    List<Double> ValuesLeft = new ArrayList<Double>();
    List<Double> ValuesRight = new ArrayList<Double>();

    double left_trigger = Xbox.getRawAxis(2);
  
    boolean Button1 = Xbox.getRawButton(1);
    boolean Button2 = Xbox.getRawButton(2);

    while (isOperatorControl() && isEnabled()) {
      //Track encoder data
      int LeftEncoder = LeftDrive.getSensorCollection().getPulseWidthPosition();
      int RightEncoder = RightDrive.getSensorCollection().getPulseWidthPosition();
      System.out.println("Left Encoder: "+ LeftEncoder + " Right Encoder: " + RightEncoder);
      Button2 = Xbox.getRawButton(2);
      Button1 = Xbox.getRawButton(1);

      //Change these to affect drive preformace
      double Sensitivity = 0.50;
      double Dead_band = 0.1;

      //This Scales the Joysticks to affect power 
      double left = Xbox.getRawAxis(1) * Sensitivity;
      double Right = Xbox.getRawAxis(5) * (Sensitivity + 0.03);

      //simple Deadband
      if(left > Dead_band || left < -(Dead_band)){
      left = Xbox.getRawAxis(1) * Sensitivity;
      } else {
        left = 0;
      }

      if(Right > Dead_band || Right < -Dead_band){
        Right = Xbox.getRawAxis(5) * (Sensitivity + 0.03);
      } else {
       Right=0;
      }
      
      left_trigger = Xbox.getRawAxis(2);

      //Taking a rolling average of the joystick imputs to smooth out movement
      LeftDrive.set(ControlMode.PercentOutput, -RollingAverage(left,ValuesLeft,25));
      RightDrive.set(ControlMode.PercentOutput, RollingAverage(Right,ValuesRight,25));

      //Modfy this to have it run your mechnism if need be
      Lift1.set(ControlMode.PercentOutput, -left_trigger);
      Lift2.set(ControlMode.PercentOutput, left_trigger);
      if(Button1){
        IntakeRight.set(1);
        IntakeLeft.set(1);
      } else if (Button2){
        IntakeRight.set(-1);
        IntakeLeft.set(-1);
      } else {
        IntakeRight.set(0);
        IntakeLeft.set(0);
      }
      Timer.delay(0.01);
    }
  }

  /**
   * Runs during test mode.
   */
  @Override

  //This mode is used to tune sensors and trouble shoot systems
  public void test() {
    LeftDrive.getSensorCollection().setPulseWidthPosition(0, 50);
    RightDrive.getSensorCollection().setPulseWidthPosition(0,50);
    
    Timer.delay(0.505);

    double initLeftEncoder = LeftDrive.getSensorCollection().getPulseWidthPosition();
    double initRightEncoder = RightDrive.getSensorCollection().getPulseWidthPosition();
    //Change this to what to how many steps it takes 
    double OneRotation = 10240;
     
    List<String> OutputInfo = new ArrayList<String>();
    while(isEnabled() && isTest()){
      double LeftMotorOutput = LeftDrive.getMotorOutputVoltage() / LeftDrive.getBusVoltage();
      double RightMotorOutput = RightDrive.getMotorOutputVoltage() / RightDrive.getBusVoltage();

      double Left_error = LeftDrive.getClosedLoopError();
      double Right_error = RightDrive.getClosedLoopError();

      LeftDrive.set(ControlMode.Velocity , 0);
      RightDrive.set(ControlMode.Velocity, 0);
      OutputInfo.add("Left Power:" + LeftMotorOutput + " Right Output:" + RightMotorOutput);
      OutputInfo.add("Left Error:" + Left_error +" Right Error:"+ Right_error);
      
      System.out.println(OutputInfo);
      Timer.delay(0.005);
  }
}
 
// this isn't in test lana (inside joke)
// This fucntion will take the average of your set continuously
//Use this to help filter stop random spikes from throwin your
//Number Set off
  public double RollingAverage(double input, List<Double> Values, int buffer){
      double Rounded_Input = Math.round((input*1000));
      if(Values.size() > buffer){
        Values.remove(0);
        Values.add(Rounded_Input);
      } else {
        Values.add(Rounded_Input);
      }

      double Total = 0;
      for(int x = 0; x < Values.size(); x++){
        Total = Total + Values.get(x);
      }
      
      double Average = (Total / buffer) ;
      
      return Math.round(Average) * 0.001;
  }

}

