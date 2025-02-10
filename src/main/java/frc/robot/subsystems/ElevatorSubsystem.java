// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Falcon
  private final TalonFX m_motor = new TalonFX(ElevatorConstants.DEVICE_ID);

  private final DutyCycleOut m_motorOut = new DutyCycleOut(0);
  private final VoltageOut m_motorVoltage = new VoltageOut(0);
  // Position unit???
  private final PositionVoltage m_positionVoltageControl = new PositionVoltage(0);
  private final PositionDutyCycle m_positionControl = new PositionDutyCycle(0);

  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    var slot0motorConfigs = m_motorConfig.Slot0;
    slot0motorConfigs.withGravityType(GravityTypeValue.Elevator_Static);
    slot0motorConfigs.kV = ElevatorConstants.kV;
    slot0motorConfigs.kS = ElevatorConstants.kS;
    slot0motorConfigs.kP = ElevatorConstants.kP;
    slot0motorConfigs.kI = ElevatorConstants.kI;
    slot0motorConfigs.kD = ElevatorConstants.kD;

    // Apply configs
    m_motor.getConfigurator().apply(slot0motorConfigs, 0.05);

  }
  
  /**
   * Incomplete.
   * Learn details on position unit and conversion and return to writing.
   * 
   * @param position
   * @return
   */
  public Command setPositionWithVoltComp(double position) {
    m_positionVoltageControl.Slot = 0;
    return run(() -> m_motor.setControl(m_positionVoltageControl.withPosition(position)));
  }

  /**
   * Incomplete.
   * Learn details on position unit and conversion and return to writing.
   * 
   * @param position
   * @return
   */
  public Command setPosition(double position) {
    m_positionControl.Slot = 0;
    return run(() -> m_motor.setControl(m_positionControl.withPosition(position)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
