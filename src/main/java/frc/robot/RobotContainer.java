// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.StatePositions;
import frc.robot.sims.MainRobotMechanism;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.MainMechSubsystem;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem();

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();

  private final MainMechSubsystem m_mainMechSubsystem = new MainMechSubsystem(m_armSubsystem, m_elevatorSubsystem, m_gripperSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_driverController =
      new CommandXboxController(1);

  private final SendableChooser<Integer> m_reefChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("CoralIntake", m_mainMechSubsystem.CoralIntakeCommand());
    NamedCommands.registerCommand("CoralStage1", m_mainMechSubsystem.CoralStage1Command());
    NamedCommands.registerCommand("CoralStage2", m_mainMechSubsystem.CoralStage2Command());
    NamedCommands.registerCommand("CoralStage3", m_mainMechSubsystem.CoralStage3Command());
    NamedCommands.registerCommand("CoralStage4", m_mainMechSubsystem.CoralStage4Command());
    NamedCommands.registerCommand("ThrowAlgaeNet", m_mainMechSubsystem.ThrowAlgaeNetCommand());
    NamedCommands.registerCommand("AlgaeGround", m_mainMechSubsystem.AlgaeGroundCommand());
    NamedCommands.registerCommand("Closed", m_mainMechSubsystem.ClosedCommand());
    NamedCommands.registerCommand("FullyClosed", m_mainMechSubsystem.FullyClosedCommand());
    NamedCommands.registerCommand("Algae23", m_mainMechSubsystem.Algae23Command());
    NamedCommands.registerCommand("Algae34", m_mainMechSubsystem.Algae34Command());
    NamedCommands.registerCommand("AlgaeCarry", m_mainMechSubsystem.AlgaeCarryCommand());


    configureBindings();

    double driveK = -0.3;
    double angleK = -0.3;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * m_drivebase.getDriveMultiplier(),
                                                                () -> m_driverController.getLeftX() * m_drivebase.getDriveMultiplier())
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(driveK)
                                                            .scaleRotation(angleK)
                                                            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                            .withControllerHeadingAxis(m_driverController::getRightX,
                                                                                        m_driverController::getRightY)
                                                            .headingWhile(true);

    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);

    if (RobotBase.isReal()) { 
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    m_reefChooser.setDefaultOption("1", 1);
    m_reefChooser.addOption("2", 2);
    m_reefChooser.addOption("3", 3);
    m_reefChooser.addOption("4", 4);
    m_reefChooser.addOption("5", 5);
    m_reefChooser.addOption("6", 6);
    SmartDashboard.putData("ReefN", m_reefChooser);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_operatorController.a().onTrue(NamedCommands.getCommand("Algae23"));
    m_operatorController.b().onTrue(NamedCommands.getCommand("Algae34"));
    m_operatorController.x().onTrue(NamedCommands.getCommand("AlgaeGround"));
    m_operatorController.y().onTrue(NamedCommands.getCommand("AlgaeCarry"));
    m_operatorController.button(5).onTrue(NamedCommands.getCommand("CoralIntake"));
    m_operatorController.button(6).onTrue(NamedCommands.getCommand("ThrowAlgaeNet"));
    m_operatorController.button(7).onTrue(NamedCommands.getCommand("AlgaeGround"));
    m_operatorController.button(8).onTrue(NamedCommands.getCommand("FullyClosed"));
    m_operatorController.button(10).onTrue(NamedCommands.getCommand("Closed"));

    //m_operatorController.button(9).onTrue(NamedCommands.getCommand("TestCommand"));
    m_operatorController.button(9).onTrue(m_gripperSubsystem.stopCommand());

    m_driverController.pov(0).whileTrue(m_gripperSubsystem.takeAlgae());
    m_driverController.pov(90).whileTrue(m_gripperSubsystem.throwAlgae());
    m_driverController.pov(180).whileTrue(m_gripperSubsystem.takeCoral());
    m_driverController.pov(270).whileTrue(m_gripperSubsystem.throwCoral());

    m_driverController.button(8).onTrue(NamedCommands.getCommand("FullyClosed"));
    m_driverController.button(10).onTrue(NamedCommands.getCommand("Closed"));
   
    /*
    m_driverController.y().whileTrue(AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(3.01,3.87),
                                                                             new Rotation2d(Units.radiansToDegrees(0))), m_drivebase.getConstraints())
                                                                             .andThen(NamedCommands.getCommand("CoralStage4"))
                                                                             .andThen(m_gripperSubsystem.throwCoral()));
    m_driverController.x().whileTrue(AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(3.04,3.9),
                                                                             new Rotation2d(Units.radiansToDegrees(0))), m_drivebase.getConstraints())
                                                                             .andThen(NamedCommands.getCommand("CoralStage3"))
                                                                             .andThen(m_gripperSubsystem.throwCoral()));
    */
                                                                             /*
    m_driverController.y().whileTrue(AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(3,4),
                                                                             new Rotation2d(Units.radiansToDegrees(0))), m_drivebase.getConstraints()));
    
    */
     
    m_driverController.y().and(() -> {return checkCoral(17);}).whileTrue(m_drivebase.goToReef(17, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return checkCoral(17);}).whileTrue(m_drivebase.goToReef(17, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return checkCoral(17);}).whileTrue(m_drivebase.goToReef(17, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return checkCoral(17);}).whileTrue(m_drivebase.goToReef(17, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return checkCoral(18);}).whileTrue(m_drivebase.goToReef(18, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return checkCoral(18);}).whileTrue(m_drivebase.goToReef(18, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return checkCoral(18);}).whileTrue(m_drivebase.goToReef(18, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return checkCoral(18);}).whileTrue(m_drivebase.goToReef(18, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return checkCoral(19);}).whileTrue(m_drivebase.goToReef(19, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return checkCoral(19);}).whileTrue(m_drivebase.goToReef(19, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return checkCoral(19);}).whileTrue(m_drivebase.goToReef(19, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return checkCoral(19);}).whileTrue(m_drivebase.goToReef(19, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return checkCoral(20);}).whileTrue(m_drivebase.goToReef(20, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return checkCoral(20);}).whileTrue(m_drivebase.goToReef(20, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return checkCoral(20);}).whileTrue(m_drivebase.goToReef(20, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return checkCoral(20);}).whileTrue(m_drivebase.goToReef(20, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return checkCoral(21);}).whileTrue(m_drivebase.goToReef(21, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return checkCoral(21);}).whileTrue(m_drivebase.goToReef(21, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return checkCoral(21);}).whileTrue(m_drivebase.goToReef(21, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return checkCoral(21);}).whileTrue(m_drivebase.goToReef(21, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return checkCoral(22);}).whileTrue(m_drivebase.goToReef(22, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return checkCoral(22);}).whileTrue(m_drivebase.goToReef(22, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return checkCoral(22);}).whileTrue(m_drivebase.goToReef(22, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return checkCoral(22);}).whileTrue(m_drivebase.goToReef(22, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    

    m_driverController.button(6).whileTrue(new RunCommand(() -> {m_drivebase.zeroGyro();}));
    m_driverController.button(5).onTrue(NamedCommands.getCommand("CoralIntake"));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public boolean checkCoral(int reefTag) {
    SmartDashboard.putBoolean("testReefN18", (m_reefChooser.getSelected() + 16) == 18);
    return (m_reefChooser.getSelected() + 16) == reefTag;
  }

  public void resetMechanisms() {
    m_armSubsystem.resetArmPositions();
    m_elevatorSubsystem.resetMotorPosition();
  }

  public void resetEncoders()
  {
    m_armSubsystem.resetEncoders();
  }

}
