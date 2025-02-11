// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/teamnf"));
  
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final GripperSubsystem m_gripper = new GripperSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    double driveK = 0.6;
    double angleK = 0.85;
    Command driveRobotOrientedAngularVelocity = m_drivebase.robotCentricDriveCommand(
        () -> (MathUtil.applyDeadband(m_driverController.getLeftY(), 0.2) * driveK),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), 0.2) * driveK,
        () -> m_driverController.getRightX() * angleK);

    Command driveFieldOrientedDirectAngle = m_drivebase.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY(), 0.2) * driveK,
      () -> MathUtil.applyDeadband(m_driverController.getLeftX(), 0.2) * driveK,
      () -> m_driverController.getRightX() * angleK,
      () -> m_driverController.getRightY() * angleK);

    m_drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    // Simulation
    if (RobotBase.isSimulation()) {
    Mechanism2d arm = new Mechanism2d(20, 20);
    MechanismRoot2d armRoot = arm.getRoot("armroot", 10, 0);

    var m_mechElevator = armRoot.append(new MechanismLigament2d("elevator", 8, 90));
    var m_mechCage = armRoot.append(new MechanismLigament2d("cage", 1, 90));
    var m_shoulder = m_mechCage.append(new MechanismLigament2d("shoulder", 5, -20));
    var m_elbow = m_shoulder.append(new MechanismLigament2d("elbow", 4, 0));
    var m_wrist = m_elbow.append(new MechanismLigament2d("wrist", 2, 15));

    SmartDashboard.putData("Mech2d", arm);
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //m_gripper.controlWithTriggers(m_driverController.getLeftTriggerAxis()).onlyIf(() -> m_driverController.getLeftTriggerAxis() > 0.2);
    //m_gripper.controlWithTriggers(-m_driverController.getRightTriggerAxis()).onlyIf(() -> m_driverController.getRightTriggerAxis() > 0.2);
    if (RobotBase.isSimulation()) {
    //m_driverController.a().onTrue(m_elevator.setPositionSimulation());
    //m_driverController.a().onFalse(m_elevator.setPosition(0.2));  
    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_drivebase);
  }
}
