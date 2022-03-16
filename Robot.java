// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;

import javax.lang.model.element.ElementVisitor;
import javax.swing.JInternalFrame.JDesktopIcon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
//import edu.wpi.first.wpilibj.Threads;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kShooterCANID = 8;
  private static final int kIntakeCANID = 4;
  private static final int kElevatorCANID = 10;

  private static final int kMotorCANID = 5;
  private static final int kJoystickPort = 0;
  private static final int kEncoderPortA = 5;
  private static final int kEncoderPortB = 4;

  private static final int kBackLeftCANID = 6;
  private static final int kBackLeftJoystickPort = 0;
  private static final int kBackLeftEncoderPortA = 3;
  private static final int kBackLeftEncoderPortB = 2;

  private static final int kFrontLeftCANID = 7;
  private static final int kFrontLeftJoystickPort = 0;
  private static final int kFrontLeftEncoderPortA = 7;
  private static final int kFrontLeftEncoderPortB = 6;

  private static final int kBackRightCANID = 9;
  private static final int kBackRightJoystickPort = 0;
  private static final int kBackRightEncoderPortA = 9;
  private static final int kBackRightEncoderPortB = 8;
  
  String auton;
 

  private VictorSPX m_motor;
  private Joystick m_joystick;
  private Encoder m_encoder;

  private VictorSPX m_FrontLeft_motor;
  private Joystick m_FrontLeft_joystick;
  private Encoder m_FrontLeft_encoder;

  private VictorSPX m_BackLeft_motor;
  private Joystick m_BackLeft_joystick;
  private Encoder m_BackLeft_encoder;

  private VictorSPX m_BackRight_motor;
  private Joystick m_BackRight_joystick;
  private Encoder m_BackRight_encoder;



   //Driving Motors

   private TalonSRX FrontRight_drive;
   private static final int kFrontRightDriveCANID = 3;
   private Joystick m_FrontRightJoystick;
   private int FrontRightJoystickPort = 1;

   private TalonSRX FrontLeft_drive;
   private static final int kFrontLeftDriveCANID = 2;
   private Joystick m_FrontLeftJoystick;
   private int FrontLeftJoystickPort = 1;

   private TalonSRX BackRight_drive;
   private static final int kBackRightDriveCANID = 0;
   private Joystick m_BackRightJoystick;
   private int BackRightJoystickPort = 1;
  
   private TalonSRX BackLeft_drive;
   private static final int kBackLeftDriveCANID = 1;
   private Joystick BackLeftJoystick;
   private int BackLeftJoystickPort = 1;
   SendableChooser<String> m_chooser;

   //shoot, elevator,intake
   private CANSparkMax m_shooter;
   private VictorSPX m_elevator;
   private CANSparkMax m_intake;
   private Joystick test_shooter;
   private int test_shooterJoyVall = 1;
   private Joystick test_elevator;
   private int test_elevatorJoyVall = 1;
   private Joystick test_intake;
   private int test_intakeJoyVall = 1;

   double allSpeed = 0;
   int time = 0;
   int timesPrinted = 0;

   double turnPosition = 0;
   double turnSpeed = 0;


   //Climing

   private Spark m_RightCliming;
   private Spark m_LeftCliming;

   // Vision
   Thread m_VisionThread;

  @Override
  public void robotInit() {
  // m_BackLeft_encoder.reset();
   //m_FrontLeft_encoder.reset();
   //m_BackRight_encoder.reset();
   //m_encoder.reset();
   
   // CAMERA STUFF. ASK IAN FOR DETAILS.
    CameraServer.startAutomaticCapture();
    CvSource outputSteam = CameraServer.putVideo("Cam1", 640, 480);
   
    SendableChooser<String> m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("AutonDefault","Default Auton");
    m_chooser.addOption("Auton1TRed", "1");
    m_chooser.addOption("Auton2TRed", "2");
    m_chooser.addOption("Auton3TRed", "3");
    m_chooser.addOption("Auton4TRed", "4");

    SmartDashboard.putData(m_chooser);

    m_motor = new VictorSPX(kMotorCANID);
    m_joystick = new Joystick(kJoystickPort);
    m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);

    m_FrontLeft_motor = new VictorSPX(kFrontLeftCANID);
    m_FrontLeft_joystick = new Joystick(kFrontLeftJoystickPort);
    m_FrontLeft_encoder = new Encoder(kFrontLeftEncoderPortA, kFrontLeftEncoderPortB);

    m_BackLeft_motor = new VictorSPX(kBackLeftCANID);
    m_BackLeft_joystick = new Joystick(kBackLeftJoystickPort);
    m_BackLeft_encoder = new Encoder(kBackLeftEncoderPortA, kBackLeftEncoderPortB);
    
    m_BackRight_motor = new VictorSPX(kBackRightCANID);
    m_BackRight_joystick = new Joystick(kBackRightJoystickPort);
    m_BackRight_encoder = new Encoder(kBackRightEncoderPortA, kBackRightEncoderPortB);
    
    //Drive Motor

    FrontRight_drive = new TalonSRX(kFrontRightDriveCANID);
    m_FrontRightJoystick = new Joystick(FrontRightJoystickPort);

    FrontLeft_drive = new TalonSRX(kFrontLeftDriveCANID);
    m_FrontLeftJoystick = new Joystick(FrontLeftJoystickPort);

    BackRight_drive = new TalonSRX(kBackRightDriveCANID);
    m_BackRightJoystick = new Joystick(BackRightJoystickPort);

    BackLeft_drive = new TalonSRX(kBackLeftDriveCANID);
    BackLeftJoystick = new Joystick(BackLeftJoystickPort);

  
    //End of Drive Motor
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    m_FrontLeft_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    m_BackLeft_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    m_BackRight_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);

   
    System.out.print("This" + m_joystick.getDirectionDegrees());
    
   // m_FrontLeft_motor.set(-m_FrontLeft_encoder.getDistance());
    //m_BackLeft_motor.set(-m_BackLeft_encoder.getDistance());
   // m_BackRight_motor.setInverted(true);
   // m_BackRight_motor.set(-m_BackRight_encoder.getDistance());
      //shoot, elevator,intake

    m_shooter = new CANSparkMax(kShooterCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_elevator = new VictorSPX(kElevatorCANID);
    m_intake = new CANSparkMax(kIntakeCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    test_shooter = new Joystick(test_shooterJoyVall);
    test_elevator = new Joystick(test_elevatorJoyVall);
    test_intake = new Joystick(test_intakeJoyVall);

//place holder PWM ports
     m_RightCliming = new Spark(0);
     m_LeftCliming = new Spark(1);


    m_motor.set(VictorSPXControlMode.PercentOutput,0);
   m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,0);
   //m_BackLeft_motor.setInverted(true);
   m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput,0);
   //m_BackRight_motor.setInverted(true);
   m_BackRight_motor.set(VictorSPXControlMode.PercentOutput,0);
    
   
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
    SmartDashboard.putNumber("Encoder", m_FrontLeft_encoder.getDistance());
    SmartDashboard.putNumber("Encoder", m_BackLeft_encoder.getDistance());
    SmartDashboard.putNumber("Encoder", m_BackRight_encoder.getDistance());

    
  }

  @Override
  public void autonomousInit() {
  //String auton = m_chooser.getSelected();

  }
  public void driveAuton(double speed){
    FrontRight_drive.set(TalonSRXControlMode.PercentOutput, speed);
    FrontLeft_drive.set(TalonSRXControlMode.PercentOutput, speed);
    BackRight_drive.set(TalonSRXControlMode.PercentOutput, speed);
    BackLeft_drive.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void turning(double turnSpeed, int turnTime) {
    m_motor.set(VictorSPXControlMode.PercentOutput,turnSpeed);
    m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,turnSpeed);
    m_BackRight_motor.set(VictorSPXControlMode.PercentOutput,turnSpeed);
    m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput,turnSpeed); 
  }
  
  public void rotate(double rightspeed, double leftspeed)
  {
    FrontRight_drive.set(TalonSRXControlMode.PercentOutput,rightspeed);
    FrontLeft_drive.set(TalonSRXControlMode.PercentOutput,leftspeed);
    BackRight_drive.set(TalonSRXControlMode.PercentOutput,rightspeed);
    BackLeft_drive.set(TalonSRXControlMode.PercentOutput,leftspeed);
  }

  public void shoot(int time, double power) throws InterruptedException{
    m_shooter.set(power);
    wait(time);
    m_shooter.set(0);
  }

  public void elevator (int time, double power) throws InterruptedException{
    m_elevator.set(VictorSPXControlMode.PercentOutput,power);
    wait(time);
    m_elevator.set(VictorSPXControlMode.PercentOutput,0);
  }

  public void intake(double power){
    m_intake.set(power);
  }
  
  /*
  public void auton1() {
    //WRITE THE AUTONOMOUS CODE, YOU FOOL
  }

  public void auton2() {
    //BOYNE WRITES A LOT HERE
  }
  */

public void Auton1TRed()throws InterruptedException{
  rotate(.9, -.9);
  wait(5000);
  driveAuton(.9);
  wait(2000);
  intake(.9);
  driveAuton(.9);
  wait(1000);
  elevator(2000, .9);
  shoot(2000,.9);
  rotate(-.9, .9);
  wait(2000);
  driveAuton(.9);
  wait(5000);
  driveAuton(.9);
  intake(.9);
  wait(2000);
  elevator(2000, .9);
  rotate(.9, -.9);
  wait(2000);
  shoot(2000, .9);
}

public void Auton2TRed() throws InterruptedException {
  Auton1TRed();
  rotate(-.9, .9);
  wait(2000);
  driveAuton(.9);
  wait(2000);
}

public void Auton3TRed() throws InterruptedException {
  driveAuton(.8);
  rotate (.9, -.9);
  wait (5000);
  driveAuton(.9);
  wait(2000);
  rotate(.9, -.9);
  wait(2000);
  driveAuton(.9);
  wait(1000);
  driveAuton(.9);
  intake(.9);
  wait(5000);
  elevator(1000, .9);
  rotate(.9, -.9); // left
  wait(2000);
  shoot(2000, .9);
}

public void Auton4TRed() throws InterruptedException {
  driveAuton(.9);
  intake(.9);
  wait(2000);
  elevator(1000, .9);
  shoot(2000, .9);
  rotate(-.9, .9); // right
  wait(2000);
  driveAuton(.9);
}
public void Auton5TRed()throws InterruptedException{
  Auton1TRed();
  rotate(-.9, .9);
  wait(2000);
  driveAuton(.9);
  wait(5000);
}
public void Auton1BBlue()throws InterruptedException{
  driveAuton(-.9);
  intake(.9);
  wait(5000);
  rotate(.9, -.9);
  wait(1000);
  elevator(1000,.9);
  shoot(2000,.9);
  rotate(.8,-.8);
  wait(2000);
}
public void Auton2BBlue()throws InterruptedException{
driveAuton(-.8);
intake(.9);
wait(2500);
rotate(-.8,.8);
wait(1000);
shoot(250,.9);
rotate(.8,-.8);
driveAuton(.8);
wait(5200);
driveAuton(.9);
intake(.9);
wait(300);
elevator(1000, .9);
rotate(-.8,.8);
wait(2000);
driveAuton(.8);
wait(2000);
shoot(250,.9);
driveAuton(.9);
intake(.9);
wait(2000);
elevator(2000, .9);
rotate(-.8, .8);
wait(1000);
shoot(250, .8);
}
public void Auton3BBlue()throws InterruptedException{
driveAuton(.8);
wait(2000);
intake(.9);
wait(2000);
elevator(2000, .9);
shoot(1000, .9);
driveAuton(.9);
wait(8000);
driveAuton(1000);
intake(.9);
wait(2000);
elevator(2000,.9);
wait(2000);
rotate(-.9,.9);
wait(2000);
shoot(1000,.9);
}
public void Auton4BBlue()throws InterruptedException{
  driveAuton(.9);
  wait(7000);
  intake(2000);
  wait(1000);
  elevator(2000, .9);
  shoot(1000, .9);
  rotate(.9, -.9);
  wait(2000);
  driveAuton(.9);
  wait(9000);
  intake(.9);
  wait(2000);
  elevator(2000,.9);
  rotate(.9,-.9);
  wait(2000);
  driveAuton(.9);
  wait(9000);
  intake(.9);
  wait(2000);
  rotate(-.9,.9);
  wait(2000);
  elevator(2000, .9);
  wait(2000);
  shoot(1000,.9);
}
public void Auton1TBlue()throws InterruptedException{
  rotate(.9, -.9);
  wait(2500);
  driveAuton(-.8);
  wait(10000);
  intake(.9);
  wait(2000);
  elevator(2000,.9);
  shoot(1000,.9);
}
public void Auton2TBlue()throws InterruptedException{
Auton1TBlue();
rotate(-.9, .9);
wait(2000);
driveAuton(.9);
wait(10000);
intake(.9);
wait(2000);
elevator(2000,.9);
wait(1000);
rotate(.9,-.9);
wait(5000);
shoot(1000,.9);
}
public void Auton3TBlue()throws InterruptedException{
Auton2TBlue();
rotate(.5,-.5);
wait(1000);
driveAuton(.8);
wait(10000);
intake(.9);
wait(1000);
elevator(2000, .9);
rotate(.5,-.5);
wait(5000);
shoot(2500, .9);
}
public void Auton4TBlue()throws InterruptedException{
  Auton1TBlue();
  wait(1000);
  driveAuton(-.8);
  rotate(.5,-.5);
  wait(7000);
  driveAuton(.9);
  wait(4000);
}
public void Auton5TBlue()throws InterruptedException{
rotate(.5,-.5);
wait(2000);
driveAuton(-.8);
wait(10000);
rotate(.9,-.9);
wait(5000);
driveAuton(-.8);
wait(8000);
intake(.9);
wait(20000);
elevator(2000,.9);
wait(2000);
rotate(-.5,.5);
wait(2000);
shoot(2000,.9);



}

public void Auton1BRed() throws InterruptedException{
rotate(0.5, 0.5);
wait(500);
rotate(0, 0);
driveAuton(-0.8);
wait(10000);
intake(0.9);
wait(2000);
driveAuton(0);
intake(0);
elevator(1000, 0.6);
shoot(1000, 0.6);

}

public void Auton2BRed() throws InterruptedException{
Auton1BRed();
driveAuton(-0.8);
wait(10000);
driveAuton(0);
}

public void Auton3BRed() throws InterruptedException{
Auton1BRed();
rotate(0.5, -0.5);
wait(2000);
rotate(0, 0);
driveAuton(0.8);
wait(10000);
intake(0.9);
wait(2000);
driveAuton(0);
intake(0);
rotate(0.5, -0.5);
wait(2500);
rotate(0, 0);
shoot(1000, 0.9);
}

public void Auton4BRed() throws InterruptedException{
rotate(0.8, -0.8);
wait(1000);
driveAuton(0.9);
wait(500);
intake(0.9);
wait(1000);
driveAuton(0);
intake(0);
elevator(750, 0.8);
rotate(0.8, -0.8);
wait(500);
shoot(750, 0.8);
rotate(0.8, -0.8);
wait(500);
driveAuton(0.9);
wait(8000);
intake(0.9);
wait(1000);
driveAuton(0);
intake(0);
elevator(750, 0.8);
rotate(0.7, -0.7);
wait(750);
shoot(750, 0.8);

}

public void Auton5BRed() throws InterruptedException{
//shoot(750, 0.8);
rotate(0.7, -0.7);
wait(500);
driveAuton(0.9);
wait(250);
intake(0.9);
wait(1000);
driveAuton(0);
intake(0);
elevator(750, 0.8);
rotate(0.8, -0.8);
wait(750);
shoot(750, 0.8);
rotate(-0.8, 0.8);
wait(750);
driveAuton(.9);
rotate(-.8,.8);
wait(2000);
shoot(750,.8);

}

public void Auton6BRed() throws InterruptedException{
rotate(-0.9, 0.9);
wait(2000);
driveAuton(0.9);
wait(5000); 
}

  @Override
  public void teleopPeriodic() {
    drive();
    //joystickMotorTest();

    // This elevator code works
    

    //m_shooter.set(m_joystick.getY());




    if(m_joystick.getRawButton(3)) 
      m_elevator.set(VictorSPXControlMode.PercentOutput ,0.8);
    else if (m_joystick.getRawButton(2)){
      m_elevator.set(VictorSPXControlMode.PercentOutput ,-0.8);
    }
    else
      m_elevator.set(VictorSPXControlMode.PercentOutput,0);
    if(m_joystick.getRawButton(4))
      m_intake.set(0.5);
    else if (m_joystick.getRawButton(5)){
        m_intake.set(-0.5);
    }
    else
      m_intake.set(0);
    if(m_joystick.getRawButton(11))
      m_shooter.set(0.8);
    else if (m_joystick.getRawButton(10)){
        m_shooter.set(0.5);
    }
    else {
      //m_shooter.set(-0.5);
      m_shooter.set(0);
    }
  }
    /*
    if(m_joystick.getRawButton(1)){
      System.out.println(m_joystick.getRawButton(1));
      m_shooter.set(0.8); // ShUwUt!
    } else {
      m_shooter.set(0);
      if (m_joystick.getRawButton(3)){
        m_elevator.set(VictorSPXControlMode.PercentOutput,0.7); // ElevatOwOr UwUp
      }else if(m_joystick.getRawButton(2)){
        m_elevator.set(VictorSPXControlMode.PercentOutput,-0.7); // ElevatOwOr down
      } else {
        m_elevator.set(VictorSPXControlMode.PercentOutput,0);
      }
      if (m_joystick.getRawButton(4)){
        m_intake.set(-0.7); // Intake
      } else if (m_joystick.getRawButton(5)) {
        m_intake.set(0.7); // OwOutake
      } else {
        m_intake.set(0);
      }
     
      
    }
    */
  
  
  
  @Override
  public void autonomousPeriodic() throws InterruptedException{
    
    /*
    if(auton == "Default Auton"){
      //run the default which might be drive forward
    } else if(auton == "1") {
      Auton1TRed();
    } else if(auton == "2") {
      Auton2TRed();
    } else if(auton == "3") {
      Auton3TRed();
    } else if(auton == "4") {
      Auton4TRed();
    } else if(auton == "5") {
      Auton5TRed();
    } else if(auton == "6") {
      Auton1BBlue();
    } else if(auton == "7") {
      Auton2BBlue();
    }else if(auton == "8") {
      Auton3BBlue();
    }else if(auton == "9") {
      Auton4BBlue();
    }
    else if(auton == "10") {
      Auton1TBlue();
    }
    else if(auton == "11") {
      Auton2TBlue();
    }else if(auton == "12") {
      Auton3TBlue();
    }else if(auton == "13") {
      Auton4TBlue();
    }else if(auton == "14") {
      Auton1BRed(); 
    }else if(auton == "15") {
      Auton2BRed();
    }else if(auton == "16"){
      Auton3BRed();
    }else if(auton == "17"){
      Auton4BRed();
    }else if(auton == "18"){
      Auton5BRed();
    }else if(auton == "19"){
      Auton6BRed();
      
    }
    */
    //public void AutonDriveBackShoot1() throws InterruptedException{
      driveAuton(0.9);
      wait(2000);
      rotate(0.5, -0.5);
      wait(500);
      shoot(4,0.8);
    //  }
  }

  public void drive() {

    //System.out.println(m_FrontLeft_joystick.getY());
    double frspeed = m_FrontRightJoystick.getTwist();
    
    //double  flspeed = 0;
    //double  brspeed = 0;
    //double  blspeed = 0;

    /*
   
    if(m_FrontLeft_encoder.getDistance() <= m_encoder.getDistance()){
      m_motor.set(VictorSPXControlMode.PercentOutput,.01);
    }
    if(m_FrontLeft_encoder.getDistance() >= m_encoder.getDistance()){
      m_motor.set(VictorSPXControlMode.PercentOutput,-.01);
    }
    if(m_BackLeft_encoder.getDistance() <= m_encoder.getDistance()){
      m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput,.01);
    }
    if(m_BackLeft_encoder.getDistance() >= m_encoder.getDistance()){
      m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput,-.01);
    }
    if(m_BackRight_encoder.getDistance() <= m_encoder.getDistance()){
      m_BackRight_motor.set(VictorSPXControlMode.PercentOutput,.01);
    }
    if(m_BackRight_encoder.getDistance() >= m_encoder.getDistance()){
      m_BackRight_motor.set(VictorSPXControlMode.PercentOutput,-.01);
    }
  
  
    if (frspeed < 0.15 && frspeed > -0.15){
      frspeed = 0;
    }

    if (frspeed > 0.9){
      frspeed = 0.9;
    }

    if (frspeed < -0.9){
      frspeed = -0.9;
    }
  
    if (allSpeed < 0.15 && allSpeed > -0.15){
      allSpeed = 0;
    }

    if (allSpeed > 0.9){
      allSpeed = 0.9;
    }

    if (allSpeed < -0.9){
      allSpeed = -0.9;
    }
    */

// Old turning
/*
if(m_joystick.getY() + .25 <=.9 || m_FrontRightJoystick.getTwist() + .25 >= -.9 && time <= 1 ){
time ++;
allSpeed = 0;
m_motor.set(VictorSPXControlMode.PercentOutput,frspeed);
}
if((m_joystick.getY() + .25 <=.1 || m_joystick.getY() + .25 >=-.1  && time > 0)){
time --;
allSpeed = m_FrontRightJoystick.getTwist();
m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,allSpeed);
//m_BackLeft_motor.setInverted(true);
m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput, allSpeed);
//m_BackRight_motor.setInverted(true);
m_BackRight_motor.set(VictorSPXControlMode.PercentOutput, allSpeed);
}
*/

  // New turning
  setTurnMotor(m_motor, m_encoder, false);
  setTurnMotor(m_FrontLeft_motor, m_FrontLeft_encoder, false);
  setTurnMotor(m_BackRight_motor, m_BackRight_encoder, false);
  setTurnMotor(m_BackLeft_motor, m_BackLeft_encoder, false);

  FrontRight_drive.set(TalonSRXControlMode.PercentOutput,m_FrontRightJoystick.getY());
  BackRight_drive.setInverted(true);
  BackRight_drive.set(TalonSRXControlMode.PercentOutput,m_BackRightJoystick.getY());
  FrontLeft_drive.set(TalonSRXControlMode.PercentOutput,m_FrontLeftJoystick.getY());
  BackLeft_drive.set(TalonSRXControlMode.PercentOutput,BackLeftJoystick.getY());


/*
    if (brspeed < 0.15 && brspeed > -0.15){
      brspeed = 0;
    }

    if (brspeed > 0.9){
      brspeed = 0.9;
    }

    if (brspeed < -0.9){
      flspeed = -0.9;
    }
     if(blspeed < 0.15 && blspeed > -0.15){
      brspeed = 0;
    }

    if (blspeed > 0.9){
      brspeed = 0.9;
    }

    if (blspeed < -0.9){
      blspeed = -0.9;
    }

    if(m_FrontLeft_encoder.getDistance() >= m_encoder.getDistance()){
      flspeed -= .1;
    }
    if(m_FrontLeft_encoder.getDistance() <= m_encoder.getDistance()){
      flspeed += .1;
    }
    if(m_BackLeft_encoder.getDistance() >= m_encoder.getDistance()){
      blspeed -= .1;
    }
    if(m_BackLeft_encoder.getDistance() <= m_encoder.getDistance()){
      blspeed += .1;
    }
    if(m_BackRight_encoder.getDistance() >= m_encoder.getDistance()){
      brspeed -= .1;
    }
    if(m_BackRight_encoder.getDistance() <= m_encoder.getDistance()){
      brspeed += .1;
    /*
    if((m_BackRight_encoder.getDistance() >= -.1) && (m_BackRight_encoder.getDistance() <= .1)){
      brspeed = 0;
    }
    if((m_FrontLeft_encoder.getDistance() >= -.1) && (m_FrontLeft_encoder.getDistance() <= .1)){
      frspeed = 0;
    }
    if((m_BackLeft_encoder.getDistance() >= -.1) && (m_BackLeft_encoder.getDistance() <= .1)){
      blspeed = 0;
    }

    */
    
   

    

/*    
    if(m_FrontLeft_encoder.getDistance() < m_BackRight_encoder.getDistance()) {
      m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,speed+0.2);
    }

    if(m_BackLeft_encoder.getDistance() < m_BackRight_encoder.getDistance()) {
      m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput,speed+0.2);
    }

    if(m_encoder.getDistance() < m_BackRight_encoder.getDistance()) {
      m_motor.set(VictorSPXControlMode.PercentOutput,speed+0.2);
    }

    //if(m_FrontLeft_encoder.getDistance() < m_BackLeft_encoder.getDistance()) {
      //m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,speed+0.2);
    //}

    if(m_BackLeft_encoder.getDistance() > m_BackRight_encoder.getDistance()) {
      m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput,speed-0.2);
    }

    if(m_FrontLeft_encoder.getDistance() > m_BackRight_encoder.getDistance()) {
      m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,speed-0.2);
    }

    if(m_encoder.getDistance() > m_BackRight_encoder.getDistance()) {
      m_motor.set(VictorSPXControlMode.PercentOutput,speed-0.2);
    }

    //if(m_FrontLeft_encoder.getDistance() > m_BackLeft_encoder.getDistance()) {
      //m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,speed-0.2);
    //}
*/
    //drive stuff
   
    

    //pivoting
    
    if(m_FrontRightJoystick.getRawButton(1)){
      FrontRight_drive.set(TalonSRXControlMode.PercentOutput,-m_FrontRightJoystick.getY());
      BackRight_drive.set(TalonSRXControlMode.PercentOutput,-m_BackRightJoystick.getY());
     
    }

    steering();

    if(m_FrontRightJoystick.getRawButton(5)) {
      correctMotor(m_FrontLeft_motor, m_FrontLeft_encoder);
      correctMotor(m_motor, m_encoder);
      correctMotor(m_BackRight_motor, m_BackRight_encoder);
      correctMotor(m_BackLeft_motor, m_BackLeft_encoder);
    }

  /*
    if(m_FrontRightJoystick.getRawButton(3)){
      FrontLeft_drive.set(TalonSRXControlMode.PercentOutput,-m_FrontLeftJoystick.getY());
      BackLeft_drive.set(TalonSRXControlMode.PercentOutput,-BackLeftJoystick.getY());
    
    }
    */
  }


 public void correctMotor(VictorSPX motor, Encoder encoder) {
    double distance = encoder.getDistance();
    double speed = 0;
    if (distance > 0) {
      speed = -.1;
    } else if (distance < 0) {
      speed = .1;
    }
    motor.set(VictorSPXControlMode.PercentOutput, speed);
 }

 public void setTurnMotor(VictorSPX motor, Encoder encoder, boolean invert) {
  double distance = encoder.getDistance();

  if (invert) {distance = -distance;}
  
  double speed = turnSpeed;
  if (distance > turnPosition) {
    speed = -speed;
    if (speed == 0) {
      speed = -0.1;
    }
  } else if (distance < turnPosition) {
    if (speed == 0){
      speed = 0.1;
    }
  }
  
  motor.set(VictorSPXControlMode.PercentOutput, speed);

  if (!(timesPrinted >= 5)) {

  System.out.println("Encoder number " + timesPrinted + ": " + encoder.getDistance());
  //timesPrinted += 1;
  }
}
      
    
  public void steering(){
    turnSpeed = java.lang.Math.abs(m_FrontRightJoystick.getTwist());
    if (turnSpeed > 0.9) {
      turnSpeed = 0.9;
    }
    double turningSpeed = m_FrontRightJoystick.getTwist();
    if (turningSpeed > 0.9) {
      turningSpeed = 0.9;
    } else if (turningSpeed < -0.9) {
      turningSpeed = -0.9;
    }
    m_motor.set(VictorSPXControlMode.PercentOutput, turningSpeed);
    turnPosition = m_encoder.getDistance();
    
  }
  public void climing(){
    if(m_joystick.getRawButton(6)){
      m_RightCliming.set(m_joystick.getY());
      m_LeftCliming.set(m_joystick.getY());
    }
    else{
      m_RightCliming.set(0);
      m_LeftCliming.set(0);
    }
  }
  
public void joystickMotorTest(){
 // m_shooter.set(test_shooter.getY());
  //m_elevator.set(VictorSPXControlMode.PercentOutput, test_elevator.getY());
  //m_intake.set(test_intake.getY());
  m_shooter.set(.9);
  m_elevator.set(VictorSPXControlMode.PercentOutput,.9);
  m_intake.set(.9);

}

}

// f in chat for our fallen lines of code