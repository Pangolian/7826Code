// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {

    private final WPI_TalonSRX m_driveMotor;
    private final WPI_VictorSPX m_turningMotor;

    public Double lamprayPosition;

    // private final PIDController m_drivePIDController = new
    // PIDController(ModuleConstants.kPModuleDriveController,
    // ModuleConstants.kIModuleDriveController, 0);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            ModuleConstants.kPModuleTurningController,
            ModuleConstants.kIModuleTurningController,
            0,
            new TrapezoidProfile.Constraints(
                    ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                    ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveEncoderReversed,
            boolean turningEncoderReversed) {
        m_driveMotor = new WPI_TalonSRX(driveMotorId);
        m_turningMotor = new WPI_VictorSPX(turningMotorId);

        m_turningMotor.setInverted(false);
        m_driveMotor.setInverted(false);
        m_turningMotor.setNeutralMode(NeutralMode.Brake);

        // config API for the Drive Motor
        TalonSRXConfiguration motorConfigs = new TalonSRXConfiguration();
        motorConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
        m_driveMotor.configAllSettings(motorConfigs);

        // config API for the Turning Motor
        VictorSPXConfiguration srxConfigs = new VictorSPXConfiguration();
        // srxConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;

        srxConfigs.feedbackNotContinuous = false;
        srxConfigs.nominalOutputForward = 0;
        srxConfigs.nominalOutputReverse = 0;
        srxConfigs.closedloopRamp = 0;
        srxConfigs.peakOutputForward = 1;
        srxConfigs.peakOutputReverse = -1;

        m_turningMotor.configAllSettings(srxConfigs);

        m_turningMotor.configAllowableClosedloopError(Constants.kPIDLoopIdx, 3, Constants.kTimeoutMs); /*
                                                                                                        * always servo
                                                                                                        */

        /* set closed loop gains in slot0 */
        m_turningMotor.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
        m_turningMotor.config_kP(Constants.kPIDLoopIdx, 1.0, Constants.kTimeoutMs);
        m_turningMotor.config_kI(Constants.kPIDLoopIdx, 0.001, Constants.kTimeoutMs);
        m_turningMotor.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);

        m_turningMotor.setSensorPhase(driveEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // public double getRate() {
    // var rawVelocity = m_driveMotor.getSelectedSensorVelocity(); // selected
    // sensor (in raw sensor units) per 100ms

    // // rev_m (motor revolutions)
    // // rev_w (wheel revolutions)

    // // encoder velocity cnts 1000 ms 1 rev_m 12.56637 in 1 meter 1 rev_w
    // // 0.000233662297 meter
    // // ---------------------- * --------- * ------------ * ----------- *
    // -----------
    // // * -------------- = ---------------------------------
    // // 100 ms 1 sec 2048 cnts 1 rev_w 39.37 in 6.67 rev_m sec

    // var metersPerSecond = rawVelocity * 0.000233662297;

    // return metersPerSecond; // meters per second
    // }

    public double getAngle() {
        // lampray is connected to the drive motor
        var rawDegLampray = m_driveMotor.getSelectedSensorPosition();

        // When using analog, there are 1024 counts
        double rotation = rawDegLampray / 1024.0;
        double angle = (rotation * 360);

        return angle; // angle in degrees
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(0, Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngle()));

        // Calculate the drive output from the drive PID controller.
        // final double driveOutput = m_drivePIDController.calculate(getRate(),
        // state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(getAngle() * (Math.PI / 180),
                state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond);
        m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
        // m_turningMotor.set(ControlMode.Position, value);

        SmartDashboard.putNumber("TurnOutput", turnOutput);
        SmartDashboard.putNumber("MPS", state.speedMetersPerSecond);
    }

    public void stop() {
        m_driveMotor.set(ControlMode.PercentOutput, 0);
        m_turningMotor.set(ControlMode.PercentOutput, 0);
    }

    public void resetEncoders() {
        m_driveMotor.setSelectedSensorPosition(0);
        m_turningMotor.setSelectedSensorPosition(0);
    }

}
