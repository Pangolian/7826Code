// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;
import java.sql.Time;

import javax.lang.model.element.ElementVisitor;
import javax.swing.JInternalFrame.JDesktopIcon;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

// VISION STUFF
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.highgui.HighGui;
import org.opencv.core.MatOfPoint;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.DriverStation;

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

  public DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  String auton;
  private double startTime;

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

  // Driving Motors

  //private TalonSRX FrontRight_drive;
  //private static final int kFrontRightDriveCANID = 3;
  public Joystick m_FrontRightJoystick;
  private int FrontRightJoystickPort = 1;

  //private TalonSRX FrontLeft_drive;
  //private static final int kFrontLeftDriveCANID = 2;
  public Joystick m_FrontLeftJoystick;
  private int FrontLeftJoystickPort = 1;
  /*
  private TalonSRX BackRight_drive;
  private static final int kBackRightDriveCANID = 0;
  public Joystick m_BackRightJoystick;
  private int BackRightJoystickPort = 1;
  
   private TalonSRX BackLeft_drive;
   private static final int kBackLeftDriveCANID = 1;
   public Joystick BackLeftJoystick;
   private int BackLeftJoystickPort = 1;
   */
  SendableChooser < String > m_chooser;

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
  //int time = 0;
  int timesPrinted = 0;

  double turnPosition = 0;
  double turnSpeed = 0;

  //Climbing

 

  // Vision
  Thread m_visionThread;

  XboxController driveController;
  XboxController buttonMonke;

  // Limelight values + vision constants
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry ta = limelightTable.getEntry("ta");
  double KpAim = -0.1; // Proportional control constant
  double KpDistance = -0.1;
  double min_aim_command = 0.05; // 0.05

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 26.4;

  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 37.0;
  
  // distance from the target to the floor
  double goalHeightInches = 84.0;

  public boolean inRange;

  Spark angleMotor;
  Spark rightArm;
  Spark leftArm;  

  @Override
  public void robotInit() {

    // m_BackLeft_encoder.reset();
    //m_FrontLeft_encoder.reset();
    //m_BackRight_encoder.reset();
    //m_encoder.reset();

    // VISION STUFF. ASK IAN FOR DETAILS.
    CameraServer.startAutomaticCapture(0);
    //CvSource cam0 = CameraServer.putVideo("Cam0", 640, 480);
    CameraServer.startAutomaticCapture(1);
    //CvSource cam1 = CameraServer.putVideo("Cam1", 640, 480);
    CameraServer.startAutomaticCapture(2);
    //CvSource cam2 = CameraServer.putVideo("Cam2", 640, 480);

    //System.out.println(x);
    //System.out.println(y);
    //System.out.println(area);

    NetworkTableEntry topWidth = Shuffleboard.getTab("LiveWindow").add("Top Width", 1).getEntry();
    NetworkTableEntry topHeight = Shuffleboard.getTab("LiveWindow").add("Top Height", 1).getEntry();
    NetworkTableEntry bottomWidth = Shuffleboard.getTab("LiveWindow").add("Bottom Width", 1).getEntry();
    NetworkTableEntry bottomHeight = Shuffleboard.getTab("LiveWindow").add("Bottom Height", 1).getEntry();
    NetworkTableEntry rlOffset = Shuffleboard.getTab("LiveWindow").add("Offset", 0).getEntry();

    /*
    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Hot Trapezoid", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                //Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(50, 205, 50), 5);
              
                // Put a trapezoid over the image
                // Trapezoid points
                List<MatOfPoint> points = new ArrayList<MatOfPoint>();
                points.add( new MatOfPoint (
                new Point(320 - topWidth.getDouble(100) + rlOffset.getDouble(0), topHeight.getDouble(100)), new Point(320 + topWidth.getDouble(100) + rlOffset.getDouble(0), topHeight.getDouble(200)),
                new Point(320 + bottomWidth.getDouble(100) + rlOffset.getDouble(0), bottomHeight.getDouble(400)), new Point(320 - bottomWidth.getDouble(100) + rlOffset.getDouble(0), bottomHeight.getDouble(400))) );
                Imgproc.polylines(mat, points, true, new Scalar(50, 205, 50), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
    */
    
    

    SendableChooser < String > m_chooser = new SendableChooser < > ();
    m_chooser.setDefaultOption("AutonDefault", "Default Auton");
    m_chooser.addOption("Auton1TRed", "1");
    m_chooser.addOption("Auton2TRed", "2");
    m_chooser.addOption("Auton3TRed", "3");
    m_chooser.addOption("Auton4TRed", "4");

    SmartDashboard.putData(m_chooser);

    //m_motor = new VictorSPX(kMotorCANID);
    m_joystick = new Joystick(kJoystickPort);
    //m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);

    //m_FrontLeft_motor = new VictorSPX(kFrontLeftCANID);
    m_FrontLeft_joystick = new Joystick(kFrontLeftJoystickPort);
    // m_FrontLeft_encoder = new Encoder(kFrontLeftEncoderPortA, kFrontLeftEncoderPortB);

    // m_BackLeft_motor = new VictorSPX(kBackLeftCANID);
    m_BackLeft_joystick = new Joystick(kBackLeftJoystickPort);
    // m_BackLeft_encoder = new Encoder(kBackLeftEncoderPortA, kBackLeftEncoderPortB);

    // m_BackRight_motor = new VictorSPX(kBackRightCANID);
    m_BackRight_joystick = new Joystick(kBackRightJoystickPort);
    // m_BackRight_encoder = new Encoder(kBackRightEncoderPortA, kBackRightEncoderPortB);

    //Drive Motor

    //FrontRight_drive = new TalonSRX(kFrontRightDriveCANID);
    m_FrontRightJoystick = new Joystick(FrontRightJoystickPort);
    /*
    FrontLeft_drive = new TalonSRX(kFrontLeftDriveCANID);
    m_FrontLeftJoystick = new Joystick(FrontLeftJoystickPort);

    BackRight_drive = new TalonSRX(kBackRightDriveCANID);
    m_BackRightJoystick = new Joystick(BackRightJoystickPort);

    BackLeft_drive = new TalonSRX(kBackLeftDriveCANID);
    BackLeftJoystick = new Joystick(BackLeftJoystickPort);
    */

    //End of Drive Motor
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    //m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    //m_FrontLeft_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    //m_BackLeft_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    //m_BackRight_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);

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

    driveController = new XboxController(1);
    buttonMonke = new XboxController(0);

    //place holder PWM ports
    angleMotor = new Spark(2);
    rightArm = new Spark(1);
    leftArm = new Spark(0);

    /// m_motor.set(VictorSPXControlMode.PercentOutput,0);
    //m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,0);
    //m_BackLeft_motor.setInverted(true);
    //m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput,0);
    //m_BackRight_motor.setInverted(true);
    //m_BackRight_motor.set(VictorSPXControlMode.PercentOutput,0);

  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
    //SmartDashboard.putNumber("Encoder", m_FrontLeft_encoder.getDistance());
    // SmartDashboard.putNumber("Encoder", m_BackLeft_encoder.getDistance());
    //SmartDashboard.putNumber("Encoder", m_BackRight_encoder.getDistance());
    SmartDashboard.putNumber("joystick", m_FrontRightJoystick.getX());
    SmartDashboard.putNumber("deadband joystick", MathUtil.applyDeadband(-m_FrontRightJoystick.getY(), OIConstants.kdeadband));

    // Convert to doubles
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("limelightX", x);
    SmartDashboard.putNumber("limelightY", y);
    SmartDashboard.putNumber("limelightArea", area);
    

    // estimating distance
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);



    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    SmartDashboard.putNumber("distance in inches from target", distanceFromLimelightToGoalInches);
    
    // Change these margins to how far you need the robot to be to score consitently
    if (distanceFromLimelightToGoalInches > 100) { // too far
      SmartDashboard.putBoolean("just right", false);
      SmartDashboard.putBoolean("too close", false);
      SmartDashboard.putBoolean("too far", true);
    } else if (distanceFromLimelightToGoalInches < 60) { // too close
      SmartDashboard.putBoolean("just right", false);
      SmartDashboard.putBoolean("too close", true);
      SmartDashboard.putBoolean("too far", false);
    } else {
      SmartDashboard.putBoolean("just right", true);
      SmartDashboard.putBoolean("too close", false);
      SmartDashboard.putBoolean("too far", false);
    }

  }

  @Override
  public void autonomousInit() {
    //Timer time = new Timer();
    startTime = Timer.getFPGATimestamp();

  }
  public void driveAuton(double speed) {
    //FrontRight_drive.set(TalonSRXControlMode.PercentOutput, speed);
    //FrontLeft_drive.set(TalonSRXControlMode.PercentOutput, speed);
    //BackRight_drive.set(TalonSRXControlMode.PercentOutput, speed);
    //BackLeft_drive.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void turning(double turnSpeed, int turnTime) {
    m_motor.set(VictorSPXControlMode.PercentOutput, turnSpeed);
    m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput, turnSpeed);
    m_BackRight_motor.set(VictorSPXControlMode.PercentOutput, turnSpeed);
    m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput, turnSpeed);
  }

  public void rotate(double rightspeed, double leftspeed) {
    //FrontRight_drive.set(TalonSRXControlMode.PercentOutput,rightspeed);
    //FrontLeft_drive.set(TalonSRXControlMode.PercentOutput,leftspeed);
    //BackRight_drive.set(TalonSRXControlMode.PercentOutput,rightspeed);
    //BackLeft_drive.set(TalonSRXControlMode.PercentOutput,leftspeed);
  }

  public void shoot(int time, double power) throws InterruptedException {
    m_shooter.set(power);
    wait(time);
    m_shooter.set(0);
  }

  public void elevator(int time, double power) throws InterruptedException {
    m_elevator.set(VictorSPXControlMode.PercentOutput, power);
    wait(time);
    m_elevator.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void intake(double power) {
    m_intake.set(power);
  }




  /*

  public void auton1() {
    //WRITE THE AUTONOMOUS CODE, YOU FOOL
  }

  public void auton2() {
    //BOYNE WRITES A LOT HERE
  }
  

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
}b
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
*/

  boolean isMovedXP = false;
  boolean isMovedXN = false;
  boolean isMovedYP = false;
  boolean isMovedYN = false;

  double speedLoss = 0.12; // Probably a better name than speedDiffusuion

  double xSpeed = 0;
  double ySpeed = 0;

  double joyX = 0;
  double joyY = 0;

  @Override
  public void teleopPeriodic() {
      deceleration();
      
      System.out.println(xSpeed + " and" + ySpeed);
      m_driveSubsystem.drive(MathUtil.applyDeadband(-driveController.getRightY(), OIConstants.kdeadband) * 2 ,MathUtil.applyDeadband(driveController.getRightX(), OIConstants.kdeadband) * 2, MathUtil.applyDeadband(driveController.getLeftX(), OIConstants.kdeadband) * 0.6, //4
        false);
      System.out.println(MathUtil.applyDeadband(driveController.getLeftX(), OIConstants.kdeadband) * 0.6);

      controler();
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      //new RunCommand(

      /*
        m_driveSubsystem.drive(
            MathUtil.applyDeadband(-m_FrontRightJoystick.getY(), OIConstants.kdeadband) * 2, //5
            MathUtil.applyDeadband(m_FrontLeft_joystick.getX(), OIConstants.kdeadband) * 2, //5
            MathUtil.applyDeadband(driveController.getLeftX(), OIConstants.kdeadband) * 0.6, //4
            false);
          */

      //joystickMotorTest();

      // This elevator code works

      //m_shooter.set(m_joystick.getY());
      /*
      if (buttonMonke.getPOV() == 90) {
        aiming();
        getToRightDistance();
      }
      */
      /*
    if (buttonMonke.getAButton()) {
        aiming();
    }  
    if(buttonMonke.getBButton()){
      superAim();
    }
    if (buttonMonke.getYButtonPressed()) {
      m_elevator.set(VictorSPXControlMode.PercentOutput, 0.8);
    } else if (buttonMonke.getXButtonPressed()) {
      m_elevator.set(VictorSPXControlMode.PercentOutput, -0.8);
    } else {
      m_elevator.set(VictorSPXControlMode.PercentOutput, 0);
    }

    if (buttonMonke.getPOV() == 0) {
      m_intake.set(0.5);
    } else if (buttonMonke.getPOV() == 180) {
      m_intake.set(-0.5);
    } else {
      m_intake.set(0);
    }

    if (buttonMonke.getRightTriggerAxis() != 0) {
      m_shooter.set(0.9);
    } else if (buttonMonke.getLeftTriggerAxis() != 0) {
      m_shooter.set(0.5);
    }
   else {
      //m_shooter.set(-0.5);
      m_shooter.set(0);
    }
  }
  */
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
   
    */
  }
  

  @Override
  public void autonomousPeriodic() {

    double time = Timer.getFPGATimestamp();
    System.out.println(time - startTime);

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      if (DriverStation.getLocation() == 1) {
        System.out.println("Red, 1");
       if(time - startTime > 12){
         m_shooter.set(0);
         m_elevator.set(VictorSPXControlMode.PercentOutput, 0);
       }
       else if(time - startTime > 10){
          m_elevator.set(VictorSPXControlMode.PercentOutput, .9);
        }
        else if(time -startTime > 8){
          aiming();
          m_intake.set(0);
        }else if(time - startTime > 7){
          m_driveSubsystem.drive(0,  0,  0, false);
        }else if(time - startTime > 3){
          m_driveSubsystem.drive(-.5,  0,  0, false);
          m_intake.set(.5);
          m_shooter.set(.9);
          angleMotor.set(0);
          rightArm.set(0);
          leftArm.set(0);
        }else{
          angleMotor.set(.5);
          rightArm.set(.4);
          leftArm.set(.4);
        }
        }
        /* if (time - startTime > 5) 
        
        if (time - startTime > 5) {
          m_elevator.set(ControlMode.PercentOutput, 0);
           m_shooter.set(0);
         } else if (time - startTime > 3) {
           m_driveSubsystem.drive(0, 0, 0, false);
           m_elevator.set(ControlMode.PercentOutput, 0.8);
         } else if (time - startTime > 0) {
           m_driveSubsystem.drive(-.4, 0, 0, false);
           m_intake.set(0.7);
           m_shooter.set(0.8);
           m_driveSubsystem.drive(0, 0, -.9, false);
           m_driveSubsystem.drive(.4, 0, 0, false);
           m_intake.set(0.7);
           m_shooter.set(0.8);
         }*/

      } else if (DriverStation.getLocation() == 2) {
        System.out.println("Red, 2");
      } else if (DriverStation.getLocation() == 3) {
        System.out.println("Red, 3");
      } else {
        System.out.println("Red, no start");
      }

      if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        if (DriverStation.getLocation() == 1) {
          System.out.println("Blue, 1");
        } else if (DriverStation.getLocation() == 2) {
          System.out.println("Blue, 2");
        } else if (DriverStation.getLocation() == 3) {
          System.out.println("Blue, 3");
        } else {
          System.out.println("Blue, no start");
        }
      }
    }
  

  
  // Working auton code?
  /*

  if (time - startTime > 5) {
          m_elevator.set(ControlMode.PercentOutput, 0);
          m_shooter.set(0);
        } else if (time - startTime > 3) {
          m_driveSubsystem.drive(0, 0, 0, false);

        }
        if (time - startTime > 5) {
          m_elevator.set(ControlMode.PercentOutput, 0);
          m_shooter.set(0);
        } else if (time - startTime > 3) {
          m_driveSubsystem.drive(0, 0, 0, false);
          m_elevator.set(ControlMode.PercentOutput, 0.8);
        } else if (time - startTime > 0) {
          m_driveSubsystem.drive(-.4, 0, 0, false);
          m_intake.set(0.7);
          m_shooter.set(0.8);
          m_driveSubsystem.drive(0, 0, -.9, false);
          m_driveSubsystem.drive(.4, 0, 0, false);
          m_intake.set(0.7);
          m_shooter.set(0.8);

  Working auton code
      if (time - startTime > 5) {
       m_ elevator.set(ControlMode.PercentOutput, 0);
        m_shooter.set(0);
      } else if (time - startTime > 3) {
        m_driveSubsystem.drive(0, 0, 0, false);
        m_elevator.set(ControlMode.PercentOutput, 0.8);
      } else if (time - startTime > 0) {
        m_driveSubsystem.drive(-.4, 0, 0, false);
        m_shooter.set(0.7);
        m_intake.set(-0.8);
      }
    }
  //drives to terminal, picks up ball and shoots
            if (time - startTime > 5) {
              m_elevator.set(ControlMode.PercentOutput, 0);
               m_shooter.set(0);
             } else if (time - startTime > 3) {
               m_driveSubsystem.drive(0, 0, 0, false);
               m_elevator.set(ControlMode.PercentOutput, 0.8);
             } else if (time - startTime > 0) {
               m_shooter.set(0.7);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(0, 0, -0.9, false)
             } else if (time - startTime > 0) {
               m_intake.set(0.8);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(0.4, 0, 0, false);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(0, 0, -0.9, false);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(-0.4, 0, 0, false);
            }else{
              angleMotor.set(.5);
              rightArm.set(.4);
              leftArm.set(.4);
             }
             }
  //picks up first three balls closest to the tarmac, and shoots all of them (one at a time)
            if (time - startTime > 5) {
              m_elevator.set(ControlMode.PercentOutput, 0);
               m_shooter.set(0);
             } else if (time - startTime > 3) {
               m_driveSubsystem.drive(0, 0, 0, false);
               m_elevator.set(ControlMode.PercentOutput, 0.8);
             } else if (time - startTime > 0) {
               m_shooter.set(0.8);
               m_intake.set(0.7);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(0, 0, -0.9, false);
             } else if (tiyme - startTime > 0) {
               m_shooter.set(0.8);
               m_intake.set(0.7);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(0.4, 0, 0, false);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(0, 0, -0.9);
             } else if (time - startTime > 0) {
               m_shooter.set(0.8);
               m_intake.set(0.7);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(-.4, 0, 0, false);
             }else{
              angleMotor.set(.5);
              rightArm.set(.4);
              leftArm.set(.4);
             }
             }
    
    //picks up closest ball to tarmac, shoots it, then goes to the Hang

             }if (time-startTime > 0) {
               m_driveSubststem.drive(-.4, 0, 0, false);
             } else if (time - startTime > 0) {
                m_driveSubsystem.drive(0, 0, -0.9, false);
             } else if (time - startTime > 0) {
               m_driveSubsystem.drive(-.4, 0, 0, false);
             } else if (time- startTime >  0) {
               m_driveSubsystem.drive(0, 0, -0.9, false);
             } else if (time - startTime > 0) {
                m_shooter.set(0.8);
                m_intake.set(0.7);
             } else if (time - startTime > 0) {
                m_driveSubsystem.drive(-0.4, 0, 0, false);
             } else if (time - startTime > 0) {
              m_elevator.set(ControlMode.PercentOutput, 0);
               m_shooter.set(0);
             } else if (time - startTime > 3) {
               m_driveSubsystem.drive(0, 0, 0, false);
               m_elevator.set(ControlMode.PercentOutput, 0.8);
            }else{
              angleMotor.set(.5);
              rightArm.set(.4);
              leftArm.set(.4);
             }
             }

    //drives out of tarmac, picks up and shoots ball closest to the tarmac, then goes by the left terminal and picks up the ball closest to that, 
    then shoots it
              if (time - startTime > 5) {
              m_elevator.set(ControlMode.PercentOutput, 0);
               m_shooter.set(0);
             } else if (time - startTime > 3) {
               m_driveSubsystem.drive(0, 0, 0, false);
               m_elevator.set(ControlMode.PercentOutput, 0.8);
             } else if (time - startTime > 0) {
               m_shooter.set(0.8);
               m_intake.set(0.7);
               m_driveSubsystem.drive(-0.4, 0, 0, false);
             } else if (time - startTime > 0) {
               m_shooter.set(0.8);
               m_intake.set(0.7);
               m_driveSubsystem.drive(.4, 0, 0, false);
             } else if (time - startTime > 0) {
                m_driveSubsystem.drive(0, 0, -.9, false);
            }else{
              angleMotor.set(.5);
              rightArm.set(.4);
              leftArm.set(.4);
             }             
             }
              */
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
  // driveAuton(0.9);
  // wait(2000);
  // rotate(0.5, -0.5);
  // wait(500);
  // shoot(4,0.8);
  //  }

  //   public void drive() {

  //     //System.out.println(m_FrontLeft_joystick.getY());
  //     double frspeed = m_FrontRightJoystick.getTwist();

  //     //double  flspeed = 0;
  //     //double  brspeed = 0;
  //     //double  blspeed = 0;

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
  m_BackLeft_motor.setInverted(true);
  m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput, allSpeed);
  m_BackRight_motor.setInverted(true);
  m_BackRight_motor.set(VictorSPXControlMode.PercentOutput, allSpeed);
  }
  */

  // New turning
  /*
   setTurnMotor(m_motor, m_encoder, false);
   setTurnMotor(m_FrontLeft_motor, m_FrontLeft_encoder, false);
   setTurnMotor(m_BackRight_motor, m_BackRight_encoder, false);
   setTurnMotor(m_BackLeft_motor, m_BackLeft_encoder, false);

   FrontRight_drive.set(TalonSRXControlMode.PercentOutput,m_FrontRightJoystick.getY());
   BackRight_drive.setInverted(true);
   BackRight_drive.set(TalonSRXControlMode.PercentOutput,m_BackRightJoystick.getY());
   FrontLeft_drive.set(TalonSRXControlMode.PercentOutput,m_FrontLeftJoystick.getY());
   BackLeft_drive.set(TalonSRXControlMode.PercentOutput,BackLeftJoystick.getY());



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

     if(m_FrontLeft_encoder.getDistance() < m_BackLeft_encoder.getDistance()) {
        m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,speed+0.2);
      }

      if(m_BackLeft_encoder.getDistance() > m_BackRight_encoder.getDistance()) {
        m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput,speed-0.2);
      }

      if(m_FrontLeft_encoder.getDistance() > m_BackRight_encoder.getDistance()) {
        m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,speed-0.2);
      }

      if(m_encoder.getDistance() > m_BackRight_encoder.getDistance()) {
        m_motor.set(VictorSPXControlMode.PercentOutput,speed-0.2);
      }

      if(m_FrontLeft_encoder.getDistance() > m_BackLeft_encoder.getDistance()) {
        m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput,speed-0.2);
      }
  */
  //drive stuff

  // hi its ian. i think there's code in "pivoting" that maybe works??? but i commented it out because it was yelling at me and i felt bad
  //pivoting
  /*
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

   
     if(m_FrontRightJoystick.getRawButton(3)){
       FrontLeft_drive.set(TalonSRXControlMode.PercentOutput,-m_FrontLeftJoystick.getY());
       BackLeft_drive.set(TalonSRXControlMode.PercentOutput,-BackLeftJoystick.getY());
    
     }
 
 }
*/

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

    if (invert) {
      distance = -distance;
    }

    double speed = turnSpeed;
    if (distance > turnPosition) {
      speed = -speed;
      if (speed == 0) {
        speed = -0.1;
      }
    } else if (distance < turnPosition) {
      if (speed == 0) {
        speed = 0.1;
      }
    }

    motor.set(VictorSPXControlMode.PercentOutput, speed);

 
  }

  public void steering() {
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
 

  public void joystickMotorTest() {
    m_shooter.set(test_shooter.getY());
    m_elevator.set(VictorSPXControlMode.PercentOutput, test_elevator.getY());
    m_intake.set(test_intake.getY());
    m_shooter.set(.9);
    m_elevator.set(VictorSPXControlMode.PercentOutput, .9);
    m_intake.set(.9);

  }
  public void deceleration() {
    joyX = driveController.getRightX();
    joyY = -driveController.getRightY();

    isMovedXP = (MathUtil.applyDeadband(joyY, OIConstants.kdeadband) > 0);

    isMovedXN = (MathUtil.applyDeadband(joyY, OIConstants.kdeadband) < 0);

    isMovedYP = (MathUtil.applyDeadband(joyX, OIConstants.kdeadband) > 0);

    isMovedYN = (MathUtil.applyDeadband(joyX, OIConstants.kdeadband) < 0);

    if (isMovedXP || isMovedXN) {
      xSpeed = MathUtil.applyDeadband(joyY, OIConstants.kdeadband) * 2;
    }
    if (isMovedYP || isMovedYN) {
      ySpeed = MathUtil.applyDeadband(joyX, OIConstants.kdeadband) * 2; //5
    }

    if (xSpeed > 0 && isMovedXP == false) {
      xSpeed -= speedLoss;
    }

    if (xSpeed < 0 && isMovedXN == false) {
      xSpeed += speedLoss;
    }

    if (ySpeed > 0 && isMovedYP == false) {
      ySpeed -= speedLoss;
    }
    if (ySpeed < 0 && isMovedYN == false) {
      ySpeed += speedLoss;
    }
  }
  double acceleration = 0.012;
  public void acceleration() {
    joyX = MathUtil.applyDeadband(-driveController.getRightY(),OIConstants.kdeadband);
    joyY = MathUtil.applyDeadband(driveController.getRightX(),OIConstants.kdeadband);
   

    if (xSpeed < joyX && !(xSpeed < 0)) {
      xSpeed += acceleration;
    } else if (xSpeed > joyX && (xSpeed < 0)) {
      xSpeed -= acceleration;
    }

    if (ySpeed < joyY && !(ySpeed < 0)) {
      ySpeed += acceleration;
    } else if (ySpeed > joyY && (ySpeed < 0)) {
      ySpeed -= acceleration;
    }
  }

  // mmmmmmmmmmmmmmmm vision
  public void aiming() {
    double heading_error = -tx.getDouble(0.0);
    double steering_adjust = 0.0;
    
    if (tx.getDouble(0.0) > 1.0) {
      steering_adjust = KpAim * heading_error - min_aim_command;
    } else if (tx.getDouble(0.0) < 1.0) {
      steering_adjust = KpAim * heading_error + min_aim_command;
    }

    m_driveSubsystem.drive(0, 0, steering_adjust * 1, false); // * 0.6

    if (MathUtil.applyDeadband(tx.getDouble(0.0), 30) == 0) {
      inRange = true;
    } else {
      inRange = false;
    }
  }
  public void superAim() {
    double heading_error = -tx.getDouble(0.0);
    double distance_error = -ty.getDouble(0.0);
    double steering_adjust = 0.0;

    if (tx.getDouble(0.0) > 1.0) {
      steering_adjust = KpAim * heading_error - min_aim_command;
    } else if(tx.getDouble(0.0) < 1.0) {
      steering_adjust = KpAim * heading_error + min_aim_command; 
    }
    
    double distance_adjust = KpDistance * distance_error;
    m_driveSubsystem.drive(distance_adjust, 0, steering_adjust * 1, false);
  }

  public double distanceFromTarget(){
    // estimating distance
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    return (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
  }
  //one button for angle up, one button for angle down, one buton for each side, and one buton for both sides
  //
  public void climbing(){
     

      
    /*
    else if(driveController.getBButton()){
      rightArm.set(.3);
    }
    else if(driveController.getYButton()){
      leftArm.set(.3);
    }
    else{
      rightArm.set(0);
      leftArm.set(0);
    }
    */
    
     
    }
    public void controler(){
   if (driveController.getBButton()) {
      aiming();
    }  
    if(driveController.getAButton()){
      superAim();
    }

    if (buttonMonke.getYButton()) {
      m_elevator.set(VictorSPXControlMode.PercentOutput, 0.8);
    } else if (buttonMonke.getAButton()) {
      m_elevator.set(VictorSPXControlMode.PercentOutput, -0.8);
    } else {
      m_elevator.set(VictorSPXControlMode.PercentOutput, 0);
    }

    if (buttonMonke.getPOV() == 90) {
      m_intake.set(0.5);
    } else if (buttonMonke.getPOV() == 270 ) {
      m_intake.set(-0.5);
    } else {
      m_intake.set(0);
    }

    if (buttonMonke.getRightTriggerAxis() != 0) {
      m_shooter.set(0.8);
    } else if (buttonMonke.getLeftTriggerAxis() != 0) {
      m_shooter.set(0.1);
    }
   else {
      m_shooter.set(0);
   
   }
   
   if(buttonMonke.getRightBumper()){
     //rightArm.set(-.4);
   }
   else if(buttonMonke.getLeftBumper()){
     rightArm.set(.4);
   }
   else{
     //rightArm.set(buttonMonke.getLeftY() * .9);
     //leftArm.set(buttonMonke.getLeftY() * .9);
   }
     
    if(driveController.getLeftBumper()){
      angleMotor.set(.2);
    }
    else if(driveController.getRightBumper()){
      angleMotor.set(-.2);
    }
    else{
      angleMotor.set(-buttonMonke.getRightY() * .9);
    }
    
  
}
}

// f in chat for our fallen lines of code
