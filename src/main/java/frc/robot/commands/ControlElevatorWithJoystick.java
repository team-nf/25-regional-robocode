// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * (Muhtemelen inline yaparım) 
 * In-line yaptım onu kullanıyoruz. Silecem bunu.
 * You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands 
 */
public class ControlElevatorWithJoystick extends Command {
  private final ElevatorSubsystem m_elevator;
  private final CommandXboxController m_controller;

  private final SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(.1);
  /** Creates a new ControlElevatorWithJoystick. */
  public ControlElevatorWithJoystick(ElevatorSubsystem elevator, CommandXboxController controller) {
    m_elevator = elevator;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Olur mu böyle hocam bilmiyorum.
    // LeftY de yukarı yapınca pozitif mi oluyor bilmediğimden böyle yaptım ama bu şekil kalırsa asansör aşağı inemez
    var pos = m_slewRateLimiter.calculate(m_controller.getLeftY() < 0 ? -m_controller.getLeftY() : m_controller.getLeftY());
    m_elevator.setPosition(pos); // command olmayan bir metod kullanabilirim daha mantıklı zaten command açtım saçma sapan.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
