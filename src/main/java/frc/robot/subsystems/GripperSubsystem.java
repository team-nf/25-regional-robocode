// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;


public class GripperSubsystem extends SubsystemBase {

  private final TalonFX m_gripperMotor = new TalonFX(GripperConstants.kGripperID);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();


  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kS = GripperConstants.kGripper_kS; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = GripperConstants.kGripper_kV; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = GripperConstants.kGripper_kP; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = GripperConstants.kGripper_kI; // No output for integrated error
    configs.Slot0.kD = GripperConstants.kGripper_kD; // No output for error derivative
    configs.Voltage.withPeakForwardVoltage(Volts.of(GripperConstants.kGripper_kPFV))
      .withPeakReverseVoltage(Volts.of(-GripperConstants.kGripper_kPFV));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself 
    configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));*/

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_gripperMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Display encoder position and velocity

  }

  public void setGripperVelocity(double velocity) {
    m_gripperMotor.setControl(m_velocityVoltage.withVelocity(velocity));
    // m_fx.setControl(m_velocityTorque.withVelocity(velocity));
  }

  public void gripperTakeAlgae() {
    setGripperVelocity(0.5);
  }

  public void gripperTakeCoral() {
    setGripperVelocity(-0.3);
  }

  public void gripperThrowAlgae() {
    setGripperVelocity(-0.6);
  }

  public void gripperThrowCoral() {
    setGripperVelocity(0.5);
  }

  public void stopGripper() {
    m_gripperMotor.setControl(m_brake);
  }
}
