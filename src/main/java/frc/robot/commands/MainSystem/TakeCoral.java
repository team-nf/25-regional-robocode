// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MainSystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sims.SimConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TakeCoral extends Command {

  ElevatorSubsystem m_elevatorSubsystem;
  ArmSubsystem m_armSubsystem;

  public TakeCoral(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_armSubsystem = armSubsystem;
    addRequirements(m_elevatorSubsystem, m_armSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.reachGoal(SimConstants.StatePositions.kRobotState5[0]);
    m_armSubsystem.reachGoal(SimConstants.StatePositions.kRobotState5[1], SimConstants.StatePositions.kRobotState5[2]);
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
