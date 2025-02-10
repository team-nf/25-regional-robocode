// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Double-Jointed Arm.
 * Two Krakens move the joints from a gearbox at the elevator.
 * 
 * Might be complicated to program, use magic motion position.
 */
public class ArmSubsystem extends SubsystemBase {
  private final TalonFX m_motorShoulder = new TalonFX(12);
  private final TalonFX m_motorElbow = new TalonFX(13);

  private final DutyCycleEncoder m_encoderShoulder = new DutyCycleEncoder(0, 360, 0); 
  private final DutyCycleEncoder m_encoderElbow = new DutyCycleEncoder(1, 360, 0);

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
