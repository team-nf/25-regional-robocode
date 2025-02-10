// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/**
 * Double-Jointed Arm.
 * Two Krakens move the joints from a gearbox at the elevator.
 * 
 * Might be complicated to program, use magic motion position.
 */
public class ArmSubsystem extends SubsystemBase {
  private final TalonFX m_motorShoulder = new TalonFX(ArmConstants.SHOULDER_DEVICE_ID);
  private final TalonFX m_motorElbow = new TalonFX(ArmConstants.ELBOW_DEVICE_ID);

  private final DutyCycleEncoder m_encoderShoulder = new DutyCycleEncoder(ArmConstants.SHOULDER_ENCODER_ID, 
                                              360, ArmConstants.SHOULDER_ENCODER_INIT); 
  private final DutyCycleEncoder m_encoderElbow = new DutyCycleEncoder(ArmConstants.ELBOW_ENCODER_ID, 
                                              360, ArmConstants.ELBOW_ENCODER_INIT);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // eklemli
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Encoder 1 Connected?", m_encoderShoulder.isConnected());
    SmartDashboard.putBoolean("Encoder 2 Connected?", m_encoderElbow.isConnected());
    SmartDashboard.putNumber("Encoder 1:", m_encoderShoulder.get());
    SmartDashboard.putNumber("Encoder 2:", m_encoderElbow.get());
  }
}
