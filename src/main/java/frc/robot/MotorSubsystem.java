// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;   //isiah commented out import invalid
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Constants.FuelConstants.CLIMB_DOWN_VOLTAGE;
import static frc.robot.Constants.FuelConstants.CLIMB_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.CLIMB_STOP_VOLTAGE;
import static frc.robot.Constants.FuelConstants.CLIMB_UP_VOLTAGE;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
  private final SparkMax m_PIDMotor = new SparkMax(CLIMB_MOTOR_ID, MotorType.kBrushless);
  // private final SparkClosedLoopController closedLoopControllerRight = m_rightMotor.getClosedLoopController();
  private final SparkClosedLoopController pidController = m_PIDMotor.getClosedLoopController();
  // private final RelativeEncoder rightEncoder = m_rightMotor.getEncoder();
  private final RelativeEncoder PIDEncoder = m_PIDMotor.getEncoder();
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  /** Creates a new ExampleSubsystem. */
  public MotorSubsystem() {
    PIDEncoder.setPosition(0);
    motorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    motorConfig.closedLoop
      .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed loop
      // slot, as it will default tro slot 0.
      .p(0.4)
      .i(0.0)
      .d(0.2)
      .outputRange(-1, 1);

    motorConfig.closedLoop.maxMotion
      // Set MAXMotion parameters for position control. We don't need to pass
      // a closed loop slot, as it will default to slot 0.
      .cruiseVelocity(2500)
      .maxAcceleration(1000)
      .allowedProfileError(0.1);

    motorConfig.smartCurrentLimit(80);

    m_PIDMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command goToPosition0() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          pidController.setSetpoint(0.0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        });
  }

  public Command climbRunUp(){
    return runOnce(
        () -> {
          m_PIDMotor.set(CLIMB_UP_VOLTAGE);
        });

  }

  public Command climbRunStop(){
    return runOnce(
        () -> {
          m_PIDMotor.set(CLIMB_STOP_VOLTAGE);
        });

  }

  public Command climbRunDown(){
    return runOnce(
        () -> {
          m_PIDMotor.set(CLIMB_DOWN_VOLTAGE);
        });

  }

  public Command goToPosition1() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          pidController.setSetpoint(125.0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
