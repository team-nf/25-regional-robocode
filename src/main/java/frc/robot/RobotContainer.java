// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.sims.MainRobotMechanism;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MainMechSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.LoadPath;
import swervelib.SwerveInputStream;

import java.nio.file.Path;
import java.lang.Exception;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
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

  private final MainMechSubsystem m_mainMechSubsystem = new MainMechSubsystem(m_armSubsystem, m_elevatorSubsystem);

  private final MainRobotMechanism m_robotMechanism = new MainRobotMechanism();

    // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
    m_driverController;
  private final CommandXboxController m_simController = new CommandXboxController(1);
  //  new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final SwerveSubsystem m_drivebase  = new SwerveSubsystem();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                           .withControllerHeadingAxis(m_driverController::getRightX,
                                                                                      m_driverController::getRightY)
                                                           .headingWhile(true);

  StructPublisher<Pose3d> elevatorStage0pub = NetworkTableInstance.getDefault()
      .getStructTopic("3dSim/eleStage0", Pose3d.struct).publish();
  StructPublisher<Pose3d> elevatorStage1pub = NetworkTableInstance.getDefault()
      .getStructTopic("3dSim/eleStage1", Pose3d.struct).publish();
  StructPublisher<Pose3d> armStage0pub = NetworkTableInstance.getDefault()
      .getStructTopic("3dSim/armStage0", Pose3d.struct).publish();
  StructPublisher<Pose3d> armStage1pub = NetworkTableInstance.getDefault()
      .getStructTopic("3dSim/armStage1", Pose3d.struct).publish();
      
  private double eleGeneralHeight = 0;
  private double eleStage0Height = 0;
  private double eleStage1Height = 0;
  private double armJ1Angle = 0;
  private double armJ2Angle = 0;

  public RobotContainer() {
    LoadPath.init();

    // Configure the trigger bindings
    NamedCommands.registerCommand("CoralIntake", m_mainMechSubsystem.CoralIntakeCommand());
    NamedCommands.registerCommand("CoralStage1", m_mainMechSubsystem.CoralStage1Command());
    NamedCommands.registerCommand("CoralStage2", m_mainMechSubsystem.CoralStage2Command());
    NamedCommands.registerCommand("CoralStage3", m_mainMechSubsystem.CoralStage3Command());
    NamedCommands.registerCommand("CoralStage4", m_mainMechSubsystem.CoralStage4Command());
    NamedCommands.registerCommand("ThrowAlgae", m_mainMechSubsystem.ThrowAlgaeCommand());
    NamedCommands.registerCommand("AlgaeGround", m_mainMechSubsystem.AlgaeGroundCommand());
    NamedCommands.registerCommand("Closed", m_mainMechSubsystem.ClosedCommand());
    configureBindings();
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
    
    m_operatorController.a().onTrue(NamedCommands.getCommand("CoralStage1"));
    m_operatorController.b().onTrue(NamedCommands.getCommand("CoralStage2"));
    m_operatorController.x().onTrue(NamedCommands.getCommand("CoralStage3"));
    m_operatorController.y().onTrue(NamedCommands.getCommand("CoralStage4"));
    m_operatorController.button(5).onTrue(NamedCommands.getCommand("CoralIntake"));
    m_operatorController.button(6).onTrue(NamedCommands.getCommand("ThrowAlgae"));
    m_operatorController.button(7).onTrue(NamedCommands.getCommand("AlgaeGround"));
    m_operatorController.button(10).onTrue(NamedCommands.getCommand("Closed"));

    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);
    
    m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

 
    // Human playerdan coral alma
    m_simController.a().onTrue(
      AutoBuilder.pathfindThenFollowPath(
        // pathleri önceden bir yere çıkarıp saklamak daha mantıklı olacak sanırım?
        LoadPath.human_player_pickup, 
       m_drivebase.getConstraints())); // sürekli constraint oluşturuyor sanırım böyle bir yerde değişkene atmak mı mantıklı yoksa hepsinin constraint farklı mı olmalı?
       // Kendini düzeltmezse visionle pose verip autobuilderdan posea path
       //AutoBuilder.pathfindToPose(null, m_drivebase.getConstraints());
       // Kendini ortalaması için ortalama pathi de yazılabilir?

    // Çalışmayabilir bilmiyorum ki bir sürü command chainledim mantığı doğru mu yaptım emin değilim
    m_simController.pov(270).onTrue(
      AutoBuilder.pathfindToPose(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(18).get().toPose2d(), 
        m_drivebase.getConstraints())
        .andThen(AutoBuilder.pathfindThenFollowPath(LoadPath.coralA, m_drivebase.getConstraints())
        .onlyWhile(m_simController.leftBumper()::getAsBoolean))
        .raceWith(AutoBuilder.pathfindThenFollowPath(LoadPath.coralB, m_drivebase.getConstraints())
        .onlyWhile(m_simController.rightBumper()::getAsBoolean)));
    m_simController.pov(225).onTrue(
      AutoBuilder.pathfindToPose(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(17).get().toPose2d(), 
        m_drivebase.getConstraints())
        .andThen(AutoBuilder.pathfindThenFollowPath(LoadPath.coralC, m_drivebase.getConstraints())
        .onlyWhile(m_simController.leftBumper()::getAsBoolean))
        .raceWith(AutoBuilder.pathfindThenFollowPath(LoadPath.coralD, m_drivebase.getConstraints())
        .onlyWhile(m_simController.rightBumper()::getAsBoolean)));
    m_simController.pov(135).onTrue(
      AutoBuilder.pathfindToPose(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(22).get().toPose2d(), 
        m_drivebase.getConstraints())
        .andThen(AutoBuilder.pathfindThenFollowPath(LoadPath.coralE, m_drivebase.getConstraints())
        .onlyWhile(m_simController.leftBumper()::getAsBoolean))
        .raceWith(AutoBuilder.pathfindThenFollowPath(LoadPath.coralF, m_drivebase.getConstraints())
        .onlyWhile(m_simController.rightBumper()::getAsBoolean)));
    m_simController.pov(90).onTrue(
      AutoBuilder.pathfindToPose(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(21).get().toPose2d(), 
        m_drivebase.getConstraints())
        .andThen(AutoBuilder.pathfindThenFollowPath(LoadPath.coralG, m_drivebase.getConstraints())
        .onlyWhile(m_simController.leftBumper()::getAsBoolean))
        .raceWith(AutoBuilder.pathfindThenFollowPath(LoadPath.coralH, m_drivebase.getConstraints())
        .onlyWhile(m_simController.rightBumper()::getAsBoolean)));
    m_simController.pov(45).onTrue(
      AutoBuilder.pathfindToPose(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(20).get().toPose2d(), 
        m_drivebase.getConstraints())
        .andThen(AutoBuilder.pathfindThenFollowPath(LoadPath.coralI, m_drivebase.getConstraints())
        .onlyWhile(m_simController.leftBumper()::getAsBoolean))
        .raceWith(AutoBuilder.pathfindThenFollowPath(LoadPath.coralJ, m_drivebase.getConstraints())
        .onlyWhile(m_simController.rightBumper()::getAsBoolean)));
    m_simController.pov(315).onTrue(
      AutoBuilder.pathfindToPose(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(19).get().toPose2d(), 
        m_drivebase.getConstraints())
        .andThen(AutoBuilder.pathfindThenFollowPath(LoadPath.coralK, m_drivebase.getConstraints())
        .onlyWhile(m_simController.leftBumper()::getAsBoolean))
        .raceWith(AutoBuilder.pathfindThenFollowPath(LoadPath.coralL, m_drivebase.getConstraints())
        .onlyWhile(m_simController.rightBumper()::getAsBoolean)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_drivebase.getAutonomousCommand("The4Auto");
  }

  public void simUpdate() {

    eleGeneralHeight = m_elevatorSubsystem.getEncoderDistance();
    eleStage0Height = (eleGeneralHeight > Constants.Elevator.kStage1Height ?  eleGeneralHeight - Constants.Elevator.kStage1Height : 0);
    eleStage1Height = eleGeneralHeight - eleStage0Height;
    armJ1Angle = m_armSubsystem.getSimAngleJ1();
    armJ2Angle = m_armSubsystem.getSimAngleJ2();

    elevatorStage0pub.set(new Pose3d(0,0, eleStage0Height, new Rotation3d(0,0,0)));
    elevatorStage1pub.set(new Pose3d(0,0, eleGeneralHeight, new Rotation3d(0,0,0)));
    armStage0pub.set(new Pose3d(Constants.Arm.FirstJoint.kSimOffsets[0], Constants.Arm.FirstJoint.kSimOffsets[1], Constants.Arm.FirstJoint.kSimOffsets[2] + eleGeneralHeight, 
        new Rotation3d(0,Units.degreesToRadians(armJ1Angle),0)));
    armStage1pub.set(new Pose3d(Constants.Arm.SecondJoint.kSimOffsets[0] + Constants.Arm.FirstJoint.kArmLength * Math.sin(Units.degreesToRadians(armJ1Angle)),
                                Constants.Arm.SecondJoint.kSimOffsets[1], 
                                Constants.Arm.SecondJoint.kSimOffsets[2] + Constants.Arm.FirstJoint.kArmLength * Math.cos(Units.degreesToRadians(armJ1Angle)) + eleGeneralHeight, 
        new Rotation3d(0,Units.degreesToRadians(armJ1Angle+ armJ2Angle-90),0)));
      
    m_robotMechanism.update(eleGeneralHeight, armJ1Angle, armJ2Angle);

    m_drivebase.getSwerveDrive().addVisionMeasurement(m_drivebase.getSwerveDrive().getSimulationDriveTrainPose().orElse(new Pose2d()), Timer.getFPGATimestamp());;
    
  }

}
