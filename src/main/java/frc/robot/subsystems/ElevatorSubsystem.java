// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.StatePositions;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
  
  private final DCMotor m_elevatorGearbox = DCMotor.getFalcon500(1);
  private final TalonFX m_motor = new TalonFX(Elevator.kMotorPort);

  private final TalonFXConfiguration m_talonConfig = new TalonFXConfiguration();
  private final PositionVoltage m_positionControl = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          Elevator.kElevatorGearing,
          Elevator.kCarriageMass,
          Elevator.kElevatorDrumRadius,
          Elevator.kMinElevatorHeightMeters,
          Elevator.kMaxElevatorHeightMeters,
          true,
          0,
          0.0,
          0.0);
  private final TalonFXSimState m_talonSim = m_motor.getSimState();

  private double elevatorHeight = 0;

  private boolean isGoalReached = false;

  /** Creates a new ElevatorSubsytem. */
  public ElevatorSubsystem() {
    m_talonConfig.Slot0.kP = Elevator.kElevatorKp; // An error of 1 rotation results in 2.4 V output
    m_talonConfig.Slot0.kI = Elevator.kElevatorKi; // No output for integrated error
    m_talonConfig.Slot0.kD = Elevator.kElevatorKd; // A velocity of 1 rps results in 0.1 V output
    m_talonConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
    .kG = Elevator.kElevatorkG;
    m_talonConfig.Voltage.withPeakForwardVoltage(Volts.of(Elevator.kVoltageLimit))
    .withPeakReverseVoltage(Volts.of(-Elevator.kVoltageLimit));
    m_talonConfig.CurrentLimits.withSupplyCurrentLimit(Elevator.kAmpLimit);

    m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = Elevator.kElevatorMMCV;
    m_talonConfig.MotionMagic.MotionMagicAcceleration = Elevator.kElevatorMMA;
    m_talonConfig.MotionMagic.MotionMagicJerk = Elevator.kElevatorMMJ;

    m_talonConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    // Apply configs
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(m_talonConfig);
      
      if (status.isOK()) break;
          
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    m_motor.setPosition(0);

    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorHeight = getEncoderDistance();
    SmartDashboard.putNumber("Elevator/ElevatorHeight", elevatorHeight);
    SmartDashboard.putBoolean("Arm/isArmReady", elevatorHeight > Elevator.kReadyPos);
    if (RobotState.isDisabled()) m_motor.setControl(m_brake);
  }

  @Override
  public void simulationPeriodic() {
    m_talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_elevatorSim.setInput(m_talonSim.getMotorVoltage());
    m_elevatorSim.update(0.020);

    m_talonSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() / (Elevator.kElevatorDrumRadius * 2 * Math.PI / Elevator.kElevatorGearing));
    m_talonSim.setRotorVelocity(m_elevatorSim.getVelocityMetersPerSecond() / (Elevator.kElevatorDrumRadius * 2 * Math.PI / Elevator.kElevatorGearing));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  public void reachGoal(double goal, boolean useMotionMagic) {
    if (!useMotionMagic) {
    m_motor.setControl(m_positionControl.withPosition(goal / (Elevator.kElevatorDrumRadius * 2 * Math.PI / Elevator.kElevatorGearing)));
    } else {reachGoal(goal);}
  }

  public void reachGoal(double goal) {
    
    if(goal < Elevator.kMinElevatorHeightMeters) goal = Elevator.kMinElevatorHeightMeters;
    else if(goal > Elevator.kMaxElevatorHeightMeters) goal = Elevator.kMaxElevatorHeightMeters;
    m_motor.setControl(m_motionMagic.withPosition(goal / (Elevator.kElevatorDrumRadius * 2 * Math.PI / Elevator.kElevatorGearing)));
    isGoalReached = (Math.abs(getElevatorHeight() - goal) < Elevator.kHeightTolerance);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_motor.setControl(m_brake);
  }

  public double getEncoderDistance() { //Linear Distance
    return m_motor.getPosition().getValueAsDouble() * (Elevator.kElevatorDrumRadius * 2 * Math.PI / Elevator.kElevatorGearing);
  }

  public double getElevatorHeight() 
  {
    return elevatorHeight;
  }
  
  public Command reachGoalCommand(double h)
  {
    return run(() -> {
      reachGoal(h);
    }).until(() -> { 
      return Math.abs(elevatorHeight - h) < Elevator.kElevatorTolerance; 
    });
  }


  public void resetMotorPosition()
  {
    m_motor.setPosition(0);
  }

  public boolean isGoalReached(){
    return isGoalReached;
  }
}
