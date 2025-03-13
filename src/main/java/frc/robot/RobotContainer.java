// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.MainMechSubsystem;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final SendableChooser<Boolean> m_isColorBlue = new SendableChooser<>();
  private final SendableChooser<Boolean> m_isAlgaeMode = new SendableChooser<>();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(m_swerve.getMaxSpeed() * 0.01).withRotationalDeadband(m_swerve.getMaxAngularRate() * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.RobotCentric drivRobotCentric = new SwerveRequest.RobotCentric()
    .withDeadband(m_swerve.getMaxSpeed() * 0.01).withRotationalDeadband(m_swerve.getMaxAngularRate() * 0.01) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 


  private final double kAngle = 0.2;
  private final double kDrive = 0.9;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("CoralIntake", m_mainMechSubsystem.CoralIntakeCommand());
    NamedCommands.registerCommand("CoralStage1", m_mainMechSubsystem.CoralStage1Command());
    NamedCommands.registerCommand("CoralStage2", m_mainMechSubsystem.CoralStage2Command());
    NamedCommands.registerCommand("CoralStage3", m_mainMechSubsystem.CoralStage3Command());
    NamedCommands.registerCommand("CoralStage4", m_mainMechSubsystem.CoralStage4Command());
    NamedCommands.registerCommand("ThrowAlgaeNet", m_mainMechSubsystem.ThrowAlgaeNetCommand());
    NamedCommands.registerCommand("ThrowAlgaeProcessor", m_mainMechSubsystem.ThrowAlgaeProcessorCommand());
    NamedCommands.registerCommand("AlgaeGround", m_mainMechSubsystem.AlgaeGroundCommand());
    NamedCommands.registerCommand("Closed", m_mainMechSubsystem.ClosedCommand());
    NamedCommands.registerCommand("FullyClosed", m_mainMechSubsystem.FullyClosedCommand());
    NamedCommands.registerCommand("Algae23", m_mainMechSubsystem.Algae23Command());
    NamedCommands.registerCommand("Algae34", m_mainMechSubsystem.Algae34Command());
    NamedCommands.registerCommand("AlgaeCarry", m_mainMechSubsystem.AlgaeCarryCommand());

    //NamedCommands.registerCommand("TakeCoralAuto", m_gripperSubsystem.TakeCoralAutoCommand());
    //NamedCommands.registerCommand("ThrowCoralAuto", m_gripperSubsystem.ThrowCoralAutoCommand());



    configureBindings();


    m_swerve.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_swerve.applyRequest(() ->
          drive.withVelocityX(-m_driverController.getLeftY() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier()* kDrive) // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverController.getLeftX() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier() * kDrive) // Drive left with negative X (left)
              .withRotationalRate(-m_driverController.getRightX() * m_swerve.getMaxAngularRate() * kAngle) // Drive counterclockwise with negative X (left)
      ));

    m_reefChooser.setDefaultOption("1", 1);
    m_reefChooser.addOption("2", 2);
    m_reefChooser.addOption("3", 3);
    m_reefChooser.addOption("4", 4);
    m_reefChooser.addOption("5", 5);
    m_reefChooser.addOption("6", 6);

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          m_isColorBlue.setDefaultOption("Red", false);
          m_isColorBlue.addOption("Blue", true);

          m_autoChooser.setDefaultOption("RedAuto", getAutonomousCommandRed());
          m_autoChooser.addOption("BlueAuto", getAutonomousCommandBlue());
        }
        if (ally.get() == Alliance.Blue) {
          m_isColorBlue.setDefaultOption("Blue", true);
          m_isColorBlue.addOption("red", false);

          m_autoChooser.setDefaultOption("BlueAuto", getAutonomousCommandBlue());
          m_autoChooser.addOption("RedAuto", getAutonomousCommandRed());
        }
    }
    else {
      m_isColorBlue.setDefaultOption("Blue", true);
      m_isColorBlue.addOption("red", false);
      m_autoChooser.setDefaultOption("BlueAuto", getAutonomousCommandBlue());
      m_autoChooser.addOption("RedAuto", getAutonomousCommandRed());
    }

    m_isAlgaeMode.setDefaultOption("Coral", false);
    m_isAlgaeMode.addOption("Algae", true);

    SmartDashboard.putData("ReefN", m_reefChooser);
    SmartDashboard.putData("ColorSelect", m_isColorBlue);
    SmartDashboard.putData("AlgaeMode", m_isAlgaeMode);

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
    m_operatorController.button(5).onTrue(NamedCommands.getCommand("ThrowAlgaeProcessor"));
    m_operatorController.button(6).onTrue(NamedCommands.getCommand("ThrowAlgaeNet"));
    m_operatorController.button(7).onTrue(NamedCommands.getCommand("Closed"));
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

    m_driverController.button(6).onTrue(m_swerve.resetHeading());
    m_driverController.button(5).onTrue(NamedCommands.getCommand("CoralIntake"));

    m_driverController.leftTrigger(0.5).whileTrue(m_swerve.applyRequest(() ->
    drivRobotCentric.withVelocityX(-m_driverController.getLeftY() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier()* kDrive * 0.3) // Drive forward with negative Y (forward)
        .withVelocityY(-m_driverController.getLeftX() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier() * kDrive * 0.3) // Drive left with negative X (left)
        .withRotationalRate(-m_driverController.getRightX() * m_swerve.getMaxAngularRate() * kAngle) // Drive counterclockwise with negative X (left)
    ));

     
    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToReef(17, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToReef(17, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToReef(17, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToReef(17, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToReef(18, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToReef(18, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToReef(18, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToReef(18, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToReef(19, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToReef(19, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToReef(19, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToReef(19, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToReef(20, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToReef(20, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToReef(20, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToReef(20, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToReef(21, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToReef(21, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToReef(21, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToReef(21, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToReef(22, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToReef(22, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToReef(22, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToReef(22, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToAlgae(17, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToAlgae(17, 2)
          .andThen(NamedCommands.getCommand("Algae23")));
    
    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToAlgae(18, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToAlgae(18, 2)
          .andThen(NamedCommands.getCommand("Algae23")));
          
    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToAlgae(19, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToAlgae(19, 2)
          .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToAlgae(20, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToAlgae(20, 2)
          .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToAlgae(21, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToAlgae(21, 2)
          .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToAlgae(22, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToAlgae(22, 2)
          .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToReef(8, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToReef(8, true, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToReef(8, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToReef(6, false, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToReef(7, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToReef(7, true, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToReef(7, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToReef(7, false, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToReef(8, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToReef(8, true, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToReef(8, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToReef(8, false, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToReef(9, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToReef(9, true, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToReef(9, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToReef(9, false, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToReef(10, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToReef(10, true, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToReef(10, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToReef(10, false, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToReef(11, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToReef(11, true, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToReef(11, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !checkAlgaeMode();}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToReef(11, false, 3)
      .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToAlgae(6, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToAlgae(6, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToAlgae(7, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToAlgae(7, 2)
      .andThen(NamedCommands.getCommand("Algae23")));
      
    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToAlgae(8, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToAlgae(8, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToAlgae(9, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToAlgae(9, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToAlgae(10, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToAlgae(10, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToAlgae(11, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return checkAlgaeMode();}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToAlgae(11, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public boolean checkReef(int reefTag, boolean isBlue) {
    if(isBlue) reefTag += 16;
    else
    {
      reefTag = 9 - reefTag;
      if(reefTag <= 5) reefTag += 6;
    }
    return (m_reefChooser.getSelected() == reefTag);
  }

  public boolean checkAlgaeMode() {
    return m_isAlgaeMode.getSelected();
  }

  public void resetMechanisms() {
    m_armSubsystem.resetArmPositions();
    m_elevatorSubsystem.resetMotorPosition();
  }

  public void resetEncoders()
  {
    m_armSubsystem.resetEncoders();
  }

  public void putSelectedReefID() {
    double reefTag = m_reefChooser.getSelected();
    boolean isBlue = m_isColorBlue.getSelected();
    if(isBlue) reefTag += 16;
    else
    {
      reefTag = 9 - reefTag;
      if(reefTag <= 5) reefTag += 6;
    }
    SmartDashboard.putNumber("SelectedReefID",  reefTag);
  }

  public Command getAutonomousCommandRed() {
    // An example command will be run in autonomous
    return null;
  }

  public Command getAutonomousCommandBlue() { 
    // An example command will be run in autonomous
    return null;
  }

  public Command getAutonomousCommand() { 
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

}
