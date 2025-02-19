// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.arm.ArmVisualizer;

public class ElevatorSubsystem extends SubsystemBase {
  // Falcon
  private final TalonFX m_motor = new TalonFX(ElevatorConstants.DEVICE_ID);


  private final DutyCycleOut m_motorOut = new DutyCycleOut(0);
  private final VoltageOut m_motorVoltage = new VoltageOut(0);
  // Position unit???
  private final PositionVoltage m_positionVoltageControl = new PositionVoltage(0);
  private final PositionDutyCycle m_positionControl = new PositionDutyCycle(0);

  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

  private final DCMotor m_falcon = DCMotor.getFalcon500(1);
  
  
  // Simulation
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
    m_falcon, 
    ElevatorConstants.GEAR_REDUCTION,
    ElevatorConstants.CARRIAGE_MASS.magnitude(),
    ElevatorConstants.DRIVING_DRUM_RADIUS,
    ElevatorConstants.MIN_HEIGHT.magnitude(), ElevatorConstants.MIN_HEIGHT.magnitude(), 
    true, 
    ElevatorConstants.MIN_HEIGHT.magnitude() * 1.2);
  private final TalonFXSimState m_motorSim = m_motor.getSimState();

  private final ArmVisualizer m_visualizer;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ArmVisualizer visualizer) {
    var slot0motorConfigs = m_motorConfig.Slot0;
    slot0motorConfigs.withGravityType(GravityTypeValue.Elevator_Static);
    slot0motorConfigs.kV = ElevatorConstants.kV;
    slot0motorConfigs.kS = ElevatorConstants.kS;
    slot0motorConfigs.kP = ElevatorConstants.kP;
    slot0motorConfigs.kI = ElevatorConstants.kI;
    slot0motorConfigs.kD = ElevatorConstants.kD;

    // Apply configs
    m_motor.getConfigurator().apply(slot0motorConfigs, 0.05);

    m_visualizer = visualizer;
  }

  public double getEncoderDistance() {
    return m_motor.getPosition().getValueAsDouble() * ElevatorConstants.CONVERSION;
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
    return run(() -> m_motor.setControl(m_positionVoltageControl.withPosition(position / ElevatorConstants.CONVERSION)));
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
    return run(() -> m_motor.setControl(m_positionControl.withPosition(position/0.125 * 11.99)));
  }

  public Command setPositionWithJoystik(double control) {
    SlewRateLimiter limiter = new SlewRateLimiter(.1);
    double pos = limiter.calculate(control);
    return run(() -> m_motor.setControl(m_positionVoltageControl.withPosition(pos/0.125 * 11.99)));
  }

  public Command setPositionSimulation(){
    return run(() -> {m_elevatorSim.setState(1.25, 1);});
  }

  /**
   * Gerekli mi bilmiyorum.
   * @param goal
   * @return
   */
  public Command reachGoalAndHold(double goal) {
    return runEnd(() -> m_motor.setControl(m_positionVoltageControl.withPosition(goal / ElevatorConstants.CONVERSION)), () -> hold());
  }
  public void hold() {
    // Gerekli mi bilmiyorum.. şimdilik boş, feedforward değerini vermem gerekiyor olabilir set 0 sorun çıkarabilir gibi hissettim.
  }

  /**
   * Sets motor speed to 0 when command is interrupted.
   * @param goal
   * @return
   */
  public Command reachGoalAndStop(double goal) {
    return runEnd(() -> m_motor.setControl(m_positionVoltageControl.withPosition(goal / ElevatorConstants.CONVERSION)), () -> stop());
  }
  public void stop() {
    m_motor.stopMotor();
  }

  public Command reachGoal(double goal) {
    return run(() -> m_motor.setControl(m_positionVoltageControl.withPosition(goal / ElevatorConstants.CONVERSION)));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (position)
    m_elevatorSim.setInput(m_motorSim.getMotorVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() / ElevatorConstants.CONVERSION); 
    m_motorSim.setRotorVelocity(m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.CONVERSION);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    SmartDashboard.putNumberArray("Elevator Simulator Output", m_elevatorSim.getOutput().getData());
    m_visualizer.update(m_elevatorSim.getPositionMeters());
  }
}
