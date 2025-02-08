// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class GripperSubsystem extends SubsystemBase {
  private final SparkMax m_vortex = new SparkMax(34, MotorType.kBrushless);
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController vortexPID;
  // encoder??

  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    // Create SparkMAX Config Object (I hate this new abundant API.)
    motorConfig = new SparkMaxConfig();

    /** 
     * Configure the encoder. We are using the encoder of NEO Vortex, therefore
     * no config is needed, yet adjusting conversion factors is needed. 
     */
    motorConfig.encoder
      .positionConversionFactor(0)
      .velocityConversionFactor(0);

    /**
     * Configure the closed loop controller. The feedback sensor is the primary encoder.
     * The closed loop controller will be used for velocity control.
     */
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0)
      .i(0)
      .d(0)
      .velocityFF(0)
      .outputRange(0, 0);

    motorConfig.closedLoop.maxMotion
    // Set MAXMotion parameters for velocity control
      .maxAcceleration(0)
      .maxVelocity(0)
      .allowedClosedLoopError(0);

    // Apply the configuration to the Spark MAX
    m_vortex.configure(motorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  
    // Initialize telemetry
    SmartDashboard.setDefaultNumber("Target Velocity", 0);

  }

  /** Used for testing */
  public Command controlWithTriggers(double input) {return run(() -> m_vortex.set(input/2));}

  public Command controlFromDashboard() {
    double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
    return run(() -> vortexPID.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl));
  }

  public Command controlFromTargetVelocity(double targetVelocity) 
  {return run(() -> vortexPID.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl));}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Telemetry
    SmartDashboard.putNumber("Current Velocity", m_vortex.getEncoder().getVelocity());
    SmartDashboard.putNumber("Encoder Position", m_vortex.getEncoder().getPosition());
  }
}
