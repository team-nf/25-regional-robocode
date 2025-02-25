// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StatePositions;

public class MainMechSubsystem extends SubsystemBase {
  /** Creates a new MainMechSubsystem. */
  private final ArmSubsystem m_armSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final GripperSubsystem m_gripperSubsystem;

  public MainMechSubsystem(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
    m_armSubsystem = armSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_gripperSubsystem = null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command CoralIntakeCommand() {
    return new ParallelCommandGroup(
    m_elevatorSubsystem.reachGoalCommand(StatePositions.kCoralIntake[0]),
    m_armSubsystem.reachGoalCommand(StatePositions.kCoralIntake[1],StatePositions.kCoralIntake[2])
    );
  }

  public Command CoralStage1Command() {
    return new ParallelCommandGroup(
    m_elevatorSubsystem.reachGoalCommand(StatePositions.kCoralStage1[0]),
    m_armSubsystem.reachGoalCommand(StatePositions.kCoralStage1[1],StatePositions.kCoralStage1[2])
    );
  }

  public Command CoralStage2Command() {
    return new ParallelCommandGroup(
    m_elevatorSubsystem.reachGoalCommand(StatePositions.kCoralStage2[0]),
    m_armSubsystem.reachGoalCommand(StatePositions.kCoralStage2[1],StatePositions.kCoralStage2[2])
    );
  }

  public Command CoralStage3Command() {
    return new ParallelCommandGroup(
    m_elevatorSubsystem.reachGoalCommand(StatePositions.kCoralStage3[0]),
    m_armSubsystem.reachGoalCommand(StatePositions.kCoralStage3[1],StatePositions.kCoralStage3[2])
    );
  }

  public Command CoralStage4Command() {
    return new ParallelCommandGroup(
    m_elevatorSubsystem.reachGoalCommand(StatePositions.kCoralStage4[0]),
    m_armSubsystem.reachGoalCommand(StatePositions.kCoralStage4[1],StatePositions.kCoralStage4[2])
    );
  }

  public Command ThrowAlgaeCommand() {
    return new ParallelCommandGroup(
    m_elevatorSubsystem.reachGoalCommand(StatePositions.kAlgaeThrow[0]),
    m_armSubsystem.reachGoalCommand(StatePositions.kAlgaeThrow[1],StatePositions.kAlgaeThrow[2])
    );
  }

  public Command AlgaeGroundCommand() {
    return new ParallelCommandGroup(
    m_elevatorSubsystem.reachGoalCommand(StatePositions.kAlgaeGround[0]),
    m_armSubsystem.reachGoalCommand(StatePositions.kAlgaeGround[1],StatePositions.kAlgaeGround[2])
    );
  }

  public Command ClosedCommand() {
    return new ParallelCommandGroup(
    m_elevatorSubsystem.reachGoalCommand(StatePositions.kClosed[0]),
    m_armSubsystem.reachGoalCommand(StatePositions.kClosed[1],StatePositions.kClosed[2])
    );
  }
}
