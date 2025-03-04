// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MainSystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.ArmTraj;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowTrajectory extends Command {
  private final ArmSubsystem m_arm;
  private final ElevatorSubsystem m_elevator;
  private ArmTraj trajectory;
  private int index;
  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(ArmSubsystem arm, ElevatorSubsystem elevator, ArmTraj trajectory) {
    m_arm = arm;
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm, m_elevator);

    this.trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_elevator.reachGoalOneShot();
    m_elevator.reachGoalExpo(trajectory.getH()[index]);
    m_arm.reachGoalJ1Expo(trajectory.getTheta()[index]);
    m_arm.reachGoalJ2Expo(trajectory.getPhi()[index]);
    this.index++;
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
