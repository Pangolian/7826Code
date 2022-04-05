// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int kTimeoutMs = 10;
    public static final int kPIDLoopIdx = 0;

    public static final class DriveConstants {

        // Serve Drive
        public static final int kFrontLeftDriveId = 2;
        public static final int kFrontLeftSwerveId = 7;

        public static final int kFrontRightDriveId = 3;
        public static final int kFrontRightSwerveId = 5;

        public static final int kRearLeftDriveId = 1;
        public static final int kRearLeftSwerveId = 6;

        public static final int kRearRightDriveId = 0;
        public static final int kRearRightSwerveId = 9;

        // Gear reduction of drive motors
        public static final double kDriveGearReduction = 6.67;
        public static final double kWheelDiameter = 4.0;

        // Gyro
        public static int kGyroId = 10;
        public static boolean kIsPigeon2_0 = false;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kRearRightDriveEncoderReversed = false;

        public static final double kTrackWidth = .5842;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = .57785;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for
        // your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double kMaxSpeedMetersPerSecond = 3;
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * 2 * Math.PI;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        public static final double kPModuleTurningController = 2.0;
        public static final double kIModuleTurningController = 0.0001;

        public static final double kPModuleDriveController = 0.38;
        public static final double kIModuleDriveController = 0.0000;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kClimberControllerPort = 1;
        public static final double kdeadband = 0.25;
    }
}
