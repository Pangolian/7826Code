// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    /** Creates a new DriveSubsystem. */
    int _updateCount = 0;

    private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveId,
            DriveConstants.kFrontLeftSwerveId, DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed);
    private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveId,
            DriveConstants.kFrontRightSwerveId, DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed);
    private final SwerveModule m_rearLeft = new SwerveModule(DriveConstants.kRearLeftDriveId,
            DriveConstants.kRearLeftSwerveId, DriveConstants.kRearLeftDriveEncoderReversed,
            DriveConstants.kRearLeftTurningEncoderReversed);
    private final SwerveModule m_rearRight = new SwerveModule(DriveConstants.kRearRightDriveId,
            DriveConstants.kRearRightSwerveId, DriveConstants.kRearRightDriveEncoderReversed,
            DriveConstants.kRearRightTurningEncoderReversed);

    public DriveSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        updateDashboard();

    }

    private void updateDashboard() {
        if (_updateCount++ > 10) {
            _updateCount = 0;

            SmartDashboard.putNumber("Drive FL Angle", m_frontLeft.getAngle());
            //SmartDashboard.putNumber("Joystick", m_FrontRightJoystick.getY());
        }
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, -ySpeed, -rot,
                                Rotation2d.fromDegrees(0))
                        : new ChassisSpeeds(xSpeed, -ySpeed, -rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    public void driveStraightX(double speed) {
        drive(speed, 0, 0, false);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }
}
