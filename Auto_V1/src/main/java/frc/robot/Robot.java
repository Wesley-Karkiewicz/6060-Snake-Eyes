package frc.robot;
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
 
 
  //Global accees to center values 
  NetworkTableEntry x;
  NetworkTableEntry y;
  NetworkTableEntry radius;
  public Robot() {
    
  }

  @Override
  public void robotInit() {
   LeftDrive.configFactoryDefault();
  // LeftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,30);
   RightDrive.configFactoryDefault();
  //LeftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,30);

  // Set up and populate the networkTable
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("SmartDashboard");
  x = table.getEntry("X");
  y = table.getEntry("Y");
  radius = table.getEntry("R");
  //System.out.print(x.getType());
  //System.out.println(y.getType());
   }
   
   
  
   @Override
  
  public void autonomous() {
      /*
      * This is a P Controller where the target is always 0
      * the P normally will settle when the ball is about 3 feet away
      * To change The Distance the Robot stops, increse or decrease NeutralOffSet
      * To affect how much the robot wants to drive towards or away from the ball
      * Increase PGain
      * 
      *The other P controller Figures out how far left or right to turn the robot
      *Works Roughly the same way, X represcents the x cordnate of the center 
      *of the targe
      */

      //Varibles for Radius P controller
      double MaxRadius = 165;
      double MinRadius = 15;
      double MaxOutR = 1;
      double MinOutR = -1;
      double NeutralOffSetR= 0.6;
      double PGainR = 0.55;

      //Varibles for X P controller
      double MaxX = 580;
      double MinX = 20;
      double Maxx = 1;
      double Minx= -1;
      double NeutralOffSetX= 0;
      double PGainX = 0.4;

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
        LeftDrive.set(ControlMode.PercentOutput,  LeftSpeed);
        RightDrive.set(ControlMode.PercentOutput, RightSpeed);

        Timer.delay(0.005);
      }
    }
  

  
  @Override
  public void operatorControl() {
   
    double left_trigger = Xbox.getRawAxis(2);
    
    boolean Button1 = Xbox.getRawButton(1);
    boolean Button2 = Xbox.getRawButton(2);
    while (isOperatorControl() && isEnabled()) {
      int LeftEncoder = LeftDrive.getSensorCollection().getPulseWidthPosition();
      int RightEncoder = RightDrive.getSensorCollection().getPulseWidthPosition();

      System.out.println("Left Encoder: "+ LeftEncoder + " Right Encoder: " + RightEncoder);
      Button2 = Xbox.getRawButton(2);
      Button1 = Xbox.getRawButton(1);

      //Change these to drive
      double Sensitivity = 0.50;
      double Dead_band = 0.1;
      //Above
      double left = Xbox.getRawAxis(1) * Sensitivity;
      double Right = Xbox.getRawAxis(5) * (Sensitivity + 0.03);

      if(left > Dead_band || left < -(Dead_band)){
      left = Xbox.getRawAxis(1) * Sensitivity;
      } else {
        left = 0;
      }

  
      if(Right > Dead_band || Right < -Dead_band){
        Right = Xbox.getRawAxis(5) * Sensitivity;
      } else {
       Right=0;
      }
      
      //System.out.println(left + " " + Right);
      //System.out.println(AveragedInput1(left) + " " + AveragedInput2(Right));
      left_trigger = Xbox.getRawAxis(2);


      LeftDrive.set(ControlMode.PercentOutput, AveragedInput1(left));
      RightDrive.set(ControlMode.PercentOutput, -AveragedInput2(Right));

     
      LeftDrive2.follow(LeftDrive);
      RightDrive2.follow(RightDrive);

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
      Timer.delay(0.005);
    }
  }

  /**
   * Runs during test mode.
   */
  @Override
  public void test() {
  }
  int buffer = 25;
  //public List<E> Values = new ArrayList<E>();
  List<Double> Values = new ArrayList<Double>();
  public double AveragedInput1(double input){
      double Rounded_Input = Math.round((input*1000));
      //System.out.println(Rounded_Input);
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
      
      return Math.round(Average) * -0.001;
  }

  List<Double> Values2 = new ArrayList<Double>();
  public double AveragedInput2(double input){
    double Rounded_Input = Math.round((input*1000));
    //System.out.println(Rounded_Input);
    if(Values2.size() > buffer){
      Values2.remove(0);
      Values2.add(Rounded_Input);
    } else {
      Values2.add(Rounded_Input);
    }

    double Total = 0;
    for(int x = 0; x < Values2.size(); x++){
      Total = Total + Values2.get(x);
    }
    
    double Average = (Total / buffer) ;
    
    return Math.round(Average) * -0.001;
}
}

