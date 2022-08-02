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
  
  XboxController driveController;
  XboxController buttonMonke;
  
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
  public Joystick m_FrontRightJoystick;
  private int FrontRightJoystickPort = 1;

  public Joystick m_FrontLeftJoystick;
  private int FrontLeftJoystickPort = 1;
  
  SendableChooser < String > m_chooser;

  // shoot, elevator, intake
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
  // int time = 0;
  int timesPrinted = 0;

  double turnPosition = 0;
  double turnSpeed = 0;

  // Vision
  Thread m_visionThread;

  // limelight values + vision constants
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry ta = limelightTable.getEntry("ta");

  double KpAim = -0.1; // proportional control constant
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
    
    // Old vision trapezoid 🥵
    /*
    NetworkTableEntry topWidth = Shuffleboard.getTab("LiveWindow").add("Top Width", 1).getEntry();
    NetworkTableEntry topHeight = Shuffleboard.getTab("LiveWindow").add("Top Height", 1).getEntry();
    NetworkTableEntry bottomWidth = Shuffleboard.getTab("LiveWindow").add("Bottom Width", 1).getEntry();
    NetworkTableEntry bottomHeight = Shuffleboard.getTab("LiveWindow").add("Bottom Height", 1).getEntry();
    NetworkTableEntry rlOffset = Shuffleboard.getTab("LiveWindow").add("Offset", 0).getEntry();
    */
    
    // Old vision proc (hot trapezoid)
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

    m_joystick = new Joystick(kJoystickPort);

    m_FrontLeft_joystick = new Joystick(kFrontLeftJoystickPort);

    m_BackLeft_joystick = new Joystick(kBackLeftJoystickPort);

    m_BackRight_joystick = new Joystick(kBackRightJoystickPort);

    //Drive Motor

    m_FrontRightJoystick = new Joystick(FrontRightJoystickPort);

    //End of Drive Motor
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    //m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    //m_FrontLeft_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    //m_BackLeft_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    //m_BackRight_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
    
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
    /*
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
    */
  }

  @Override
  public void autonomousInit() {
    //Timer time = new Timer();
    startTime = Timer.getFPGATimestamp();

  }

  public void turning(double turnSpeed, int turnTime) {
    m_motor.set(VictorSPXControlMode.PercentOutput, turnSpeed);
    m_FrontLeft_motor.set(VictorSPXControlMode.PercentOutput, turnSpeed);
    m_BackRight_motor.set(VictorSPXControlMode.PercentOutput, turnSpeed);
    m_BackLeft_motor.set(VictorSPXControlMode.PercentOutput, turnSpeed);
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

  // Old auton format
  /*
  public void auton1() {
    //WRITE THE AUTONOMOUS CODE, YOU FOOL
  }

  public void auton2() {
    //BOYNE WRITES A LOT HERE
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
      m_driveSubsystem.drive(MathUtil.applyDeadband(-driveController.getRightY(), OIConstants.kdeadband) * 2 ,MathUtil.applyDeadband(driveController.getRightX(), OIConstants.kdeadband) * 2, MathUtil.applyDeadband(driveController.getLeftX(), OIConstants.kdeadband) * .8, //4
        false);
      System.out.println(MathUtil.applyDeadband(driveController.getLeftX(), OIConstants.kdeadband) * 0.6);
      //2, 2 .6

      controler();
      // TwT
  }
  

  @Override
  public void autonomousPeriodic() {
    
    double time = Timer.getFPGATimestamp();
    System.out.println(time - startTime);
    
    // Emergency default auton
    /* 
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
    }
    */    
    
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      // 1, Red
      if (DriverStation.getLocation() == 1) {
        System.out.println("Red, 1");
        if (time - startTime > 12) {
          m_shooter.set(0);
          m_elevator.set(VictorSPXControlMode.PercentOutput, 0);
        } else if(time - startTime > 10) {
          m_elevator.set(VictorSPXControlMode.PercentOutput, .9);
        } else if (time -startTime > 8) {
          aiming();
          m_intake.set(0);
        } else if (time - startTime > 7) {
          m_driveSubsystem.drive(0,  0,  0, false);
        } else if (time - startTime > 3) {
          m_driveSubsystem.drive(-.5,  0,  0, false);
          m_intake.set(.5);
          m_shooter.set(.9);
          angleMotor.set(0);
          rightArm.set(0);
          leftArm.set(0);
        } else {
          angleMotor.set(.5);
          rightArm.set(.4);
          leftArm.set(.4);
        }
      }
    
      // 2, Red
      else if (DriverStation.getLocation() == 2) {
        System.out.println("Red, 2");
      }
      // 3, Red
      else if (DriverStation.getLocation() == 3) {
        System.out.println("Red, 3");
      }
      // No red
      else {
        System.out.println("Red, no start");
      }
    
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      // 1, Blue
      if (DriverStation.getLocation() == 1) {
        System.out.println("Blue, 1");
      } 
      // 2, Blue
      else if (DriverStation.getLocation() == 2) {
        System.out.println("Blue, 2");
      } 
      // 3, Blue
      else if (DriverStation.getLocation() == 3) {
        System.out.println("Blue, 3");
      }
      // No blue
      else {
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
              }
  
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
    } else if(tx.getDouble(0.0) < -1.0) {
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

    // calculate distance
    return (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
  }
  //one button for angle up, one button for angle down, one buton for each side, and one buton for both sides
    
  public void controler(){
   if (driveController.getBButton()) {
  
  // one button for angle up, one button for angle down, one buton for each side, and one buton for both sides
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
    if (driveController.getAButton()) {
      superAim();
    }
    
   
    if (buttonMonke.getYButton()) {
      m_elevator.set(VictorSPXControlMode.PercentOutput, 0.4);
    } else if (buttonMonke.getAButton()) {
      m_elevator.set(VictorSPXControlMode.PercentOutput, -0.4);
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
      m_shooter.set(0.2);
    } else if (buttonMonke.getLeftTriggerAxis() != 0) {
      m_shooter.set(0.1);
    } else {
      m_shooter.set(0);
    }
   
    if (buttonMonke.getRightBumper()) {
      rightArm.set(-.4);
    } else if (buttonMonke.getLeftBumper()) {
     rightArm.set(.4);
    } else {
     rightArm.set(buttonMonke.getLeftY() * .9);
     leftArm.set(buttonMonke.getLeftY() * .9);
    }
     
    if(driveController.getLeftBumper()) {
      angleMotor.set(.2);
    }
    else if(driveController.getRightBumper()) {
      angleMotor.set(-.2);
    } else {
      angleMotor.set(-buttonMonke.getRightY() * .9);
    }
  }
}

// f in chat for our fallen lines of code
