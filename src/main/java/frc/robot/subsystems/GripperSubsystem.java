// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;


public class GripperSubsystem extends SubsystemBase {
  private final TalonFX m_talon = new TalonFX(GripperConstants.DEVICE_ID);
  private final DCMotor m_motor = DCMotor.getKrakenX60(1);
  
  private final VoltageOut m_voltageOut = new VoltageOut(0);
  private final VelocityVoltage m_velocityVoltageControl = new VelocityVoltage(0);
  private final NeutralOut m_brake = new NeutralOut();

  private final DigitalInput m_algeaSensor = new DigitalInput(2);
  private final DigitalInput m_coralSensor = new DigitalInput(3);

  /** 
   * Creates a new GripperSubsystem. 
   * Valla tuna ercan senden kopyaladım.
   */
  public GripperSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kS = GripperConstants.kS; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = GripperConstants.kV; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = GripperConstants.kP; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = GripperConstants.kI; // No output for integrated error
    configs.Slot0.kD = GripperConstants.kD; // No output for error derivative
    configs.Voltage.withPeakForwardVoltage(Volts.of(GripperConstants.kPFV))
      .withPeakReverseVoltage(Volts.of(-GripperConstants.kPFV));

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
      status = m_talon.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

  
    // Initialize telemetry
    SmartDashboard.setDefaultNumber("Target Velocity", 0);

  }

  /** Used for testing */
  //public Command controlWithTriggers(double input) {return run(() -> );}

  public Command controlFromDashboard() {
    double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
    return run(() -> m_talon.setControl(m_velocityVoltageControl.withVelocity(targetVelocity)));
  }

  // Maybe use motion magic?? Test first.

  public Command controlFromTargetVelocity(double targetVelocity) 
  {return run(() -> m_talon.setControl(m_velocityVoltageControl.withVelocity(targetVelocity)));}

  public Command takeAlgae() {return run(() -> m_talon.setControl(m_velocityVoltageControl.withVelocity(0.5)));}

  public Command takeCoral() {return run(() -> m_talon.setControl(m_velocityVoltageControl.withVelocity(-0.3)));}

  public Command throwAlgae() {return run(() -> m_talon.setControl(m_velocityVoltageControl.withVelocity(-0.6)));}

  public Command throwCoral() {return run(() -> m_talon.setControl(m_velocityVoltageControl.withVelocity(0.5)));}

  public Command stop() {return run(() -> m_talon.setControl(m_velocityVoltageControl.withVelocity(0.0)));}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Telemetry
    SmartDashboard.putNumber("Current Velocity", m_talon.getVelocity().getValueAsDouble());

    SmartDashboard.putData("Has Coral?: ", m_coralSensor);
    SmartDashboard.putData("Has Algea?: ", m_algeaSensor);
  }
}