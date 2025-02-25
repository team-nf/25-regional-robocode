// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Elevator;
import frc.robot.custom.ArmHalfEncoder;
import frc.robot.custom.ArmHalfEncoderSim;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_armFirstJointMotor = new TalonFX(Arm.FirstJoint.kMotorPort);
  private final TalonFX m_armSecondJointMotor = new TalonFX(Arm.SecondJoint.kMotorPort);
  private final PositionVoltage m_firstJointPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage m_secondJointPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final NeutralOut m_firstJointBrake = new NeutralOut();
  private final NeutralOut m_secondJointBrake = new NeutralOut();

  private final ArmHalfEncoder m_firstJointHalfcoder = new ArmHalfEncoder(Arm.FirstJoint.kEncoderChannel);
  private final ArmHalfEncoder m_secondJointHalfcoder = new ArmHalfEncoder(Arm.SecondJoint.kEncoderChannel);

  private final DCMotor armFirstJointDC = DCMotor.getKrakenX60(1);
  private final DCMotor armSecondJointDC = DCMotor.getKrakenX60(1);

  private final SingleJointedArmSim m_armSimJ1 =
      new SingleJointedArmSim(
          armFirstJointDC,
          Arm.FirstJoint.kArmReduction,
          SingleJointedArmSim.estimateMOI(Arm.FirstJoint.kArmLength, Arm.FirstJoint.kArmMass),
          Arm.FirstJoint.kArmLength,
          Arm.FirstJoint.kMinAngleRads,
          Arm.FirstJoint.kMaxAngleRads,
          true,
          0,
          Arm.FirstJoint.kArmEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
      );

  private final SingleJointedArmSim m_armSimJ2 =
      new SingleJointedArmSim(
          armSecondJointDC,
          Arm.SecondJoint.kArmReduction,
          SingleJointedArmSim.estimateMOI(Arm.SecondJoint.kArmLength, Arm.SecondJoint.kArmMass),
          Arm.SecondJoint.kArmLength,
          Arm.SecondJoint.kMinAngleRads,
          Arm.SecondJoint.kMaxAngleRads,
          true,
          0,
          Arm.SecondJoint.kArmEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
      );

  private final ArmHalfEncoderSim m_firstJointHalfcoderSim = new ArmHalfEncoderSim(m_firstJointHalfcoder);
  private final ArmHalfEncoderSim m_secondJointHalfcoderSim = new ArmHalfEncoderSim(m_secondJointHalfcoder);

  private final TalonFXSimState m_armFirstJointMotorSim;
  private final TalonFXSimState m_armSecondJointMotorSim;

  private double firstJointAngle = 0;
  private double secondJointAngle = 0;

  public ArmSubsystem() {
        TalonFXConfiguration firstJointConfigs = new TalonFXConfiguration();
        firstJointConfigs.Slot0.kS = Arm.FirstJoint.kArmJoint1_kS;
        firstJointConfigs.Slot0.kV = Arm.FirstJoint.kArmJoint1_kV;
        firstJointConfigs.Slot0.kP = Arm.FirstJoint.kArmJoint1_kP;
        firstJointConfigs.Slot0.kI = Arm.FirstJoint.kArmJoint1_kI;
        firstJointConfigs.Slot0.kD = Arm.FirstJoint.kArmJoint1_kD;
        firstJointConfigs.Slot0.kG = Arm.FirstJoint.kArmJoint1_kG;
        firstJointConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        firstJointConfigs.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        firstJointConfigs.Voltage.withPeakForwardVoltage(Volts.of(Arm.FirstJoint.kArmJoint1_kPFV))
            .withPeakReverseVoltage(Volts.of(-Arm.FirstJoint.kArmJoint1_kPFV));
        firstJointConfigs.CurrentLimits.withSupplyCurrentLimit(Amps.of(Arm.FirstJoint.kArmJoint1_kSCL))
            .withSupplyCurrentLowerLimit(Amps.of(Arm.FirstJoint.kArmJoint1_kSCLL));

        TalonFXConfiguration secondJointConfigs = new TalonFXConfiguration();
        secondJointConfigs.Slot0.kS = Arm.SecondJoint.kArmJoint2_kS;
        secondJointConfigs.Slot0.kV = Arm.SecondJoint.kArmJoint2_kV;
        secondJointConfigs.Slot0.kP = Arm.SecondJoint.kArmJoint2_kP;
        secondJointConfigs.Slot0.kI = Arm.SecondJoint.kArmJoint2_kI;
        secondJointConfigs.Slot0.kD = Arm.SecondJoint.kArmJoint2_kD;
        secondJointConfigs.Slot0.kG = Arm.SecondJoint.kArmJoint2_kG;
        secondJointConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        secondJointConfigs.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        secondJointConfigs.Voltage.withPeakForwardVoltage(Volts.of(Arm.SecondJoint.kArmJoint2_kPFV))
            .withPeakReverseVoltage(Volts.of(-Arm.SecondJoint.kArmJoint2_kPFV));
        secondJointConfigs.CurrentLimits.withSupplyCurrentLimit(Amps.of(Arm.SecondJoint.kArmJoint2_kSCL))
            .withSupplyCurrentLowerLimit(Amps.of(Arm.SecondJoint.kArmJoint2_kSCLL));

        StatusCode statusFirstJoint = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusFirstJoint = m_armFirstJointMotor.getConfigurator().apply(firstJointConfigs);
            if (statusFirstJoint.isOK()) break;
        }
        if (!statusFirstJoint.isOK()) {
            System.out.println("Could not apply configs, error code: " + statusFirstJoint.toString());
        }

        StatusCode statusSecondJoint = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusSecondJoint = m_armSecondJointMotor.getConfigurator().apply(secondJointConfigs);
            if (statusSecondJoint.isOK()) break;
        }
        if (!statusSecondJoint.isOK()) {
            System.out.println("Could not apply configs, error code: " + statusSecondJoint.toString());
        }

        m_armFirstJointMotorSim = m_armFirstJointMotor.getSimState();
        m_armSecondJointMotorSim = m_armSecondJointMotor.getSimState();

        m_armFirstJointMotor.setPosition(0);
        m_armSecondJointMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_firstJointHalfcoder.periodic();
    m_secondJointHalfcoder.periodic();

    
  }

  @Override
  public void simulationPeriodic(){
    m_armSimJ1.setInput(m_armFirstJointMotorSim.getMotorVoltage());
    m_armSimJ1.update(0.02);

    m_armSimJ2.setInput(m_armSecondJointMotorSim.getMotorVoltage());
    m_armSimJ2.update(0.02);

    m_firstJointHalfcoderSim.setJointAngle(Units.radiansToDegrees(m_armSimJ1.getAngleRads()) + (m_armSimJ1.getAngleRads()<0 ? 360 : 0));
    m_secondJointHalfcoderSim.setJointAngle(Units.radiansToDegrees(m_armSimJ2.getAngleRads()) + (m_armSimJ2.getAngleRads()<0 ? 360 : 0));

    m_armFirstJointMotorSim.setRawRotorPosition(Units.radiansToRotations(m_armSimJ1.getAngleRads())*Arm.FirstJoint.kArmReduction);
    m_armSecondJointMotorSim.setRawRotorPosition(Units.radiansToRotations(m_armSimJ2.getAngleRads())*Arm.SecondJoint.kArmReduction);


    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSimJ1.getCurrentDrawAmps() + m_armSimJ2.getCurrentDrawAmps()));
  
    SmartDashboard.putNumber("Arm Angle J1", Units.radiansToDegrees(m_armSimJ1.getAngleRads()));
    SmartDashboard.putNumber("Arm Angle J2", Units.radiansToDegrees(m_armSimJ2.getAngleRads()));

    firstJointAngle  = Units.radiansToDegrees(m_armSimJ1.getAngleRads());
    secondJointAngle = Units.radiansToDegrees(m_armSimJ2.getAngleRads());
  }

  public void reachGoal(double goalJ1, double goalJ2)
  {
    reachGoalJ1(goalJ1);
    reachGoalJ2(goalJ2);
  }

  public void reachGoalJ1(double goalJ1)
  {
    m_armFirstJointMotor.setControl(m_firstJointPositionVoltage.withPosition(Units.degreesToRotations(goalJ1)
     *Arm.FirstJoint.kArmReduction));
  }

  public void reachGoalJ2(double goalJ2)
  {
    m_armSecondJointMotor.setControl(m_secondJointPositionVoltage.withPosition(Units.degreesToRotations(goalJ2)
    *Arm.SecondJoint.kArmReduction));
  }

  public void brakeFirstJoint(){
    m_armFirstJointMotor.setControl(m_firstJointBrake);
  }

  public void brakeSecondJoint(){
    m_armSecondJointMotor.setControl(m_secondJointBrake);
  }

  public double getSimAngleJ1(){
    return Units.radiansToDegrees(m_armSimJ1.getAngleRads());
  }

  public double getSimAngleJ2(){
    return Units.radiansToDegrees(m_armSimJ2.getAngleRads());
  }

  public Command reachGoalJ1Command(double angleJ1)
  {
    return run(() -> {
      reachGoalJ1(angleJ1);
    });
  }

  public Command reachGoalJ2Command(double angleJ2)
  {
    return run(() -> {
      reachGoalJ2(angleJ2);
    });
  }

  public Command reachGoalCommand(double angleJ1, double angleJ2)
  {
    return run(() -> {
      reachGoal(angleJ1, angleJ2);
    }).until(() -> { 
      return Math.abs(firstJointAngle - angleJ1) < Arm.FirstJoint.kAngleTolerance && Math.abs(secondJointAngle - angleJ2) < Arm.SecondJoint.kAngleTolerance; 
    });
  }

}
