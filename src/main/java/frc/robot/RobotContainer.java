// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.sims.MainRobotMechanism;
import frc.robot.commands.MainSystem.PutCoralStage1;
import frc.robot.commands.MainSystem.PutCoralStage2;
import frc.robot.commands.MainSystem.PutCoralStage3;
import frc.robot.commands.MainSystem.PutCoralStage4;
import frc.robot.commands.MainSystem.ShootAlgae;
import frc.robot.commands.MainSystem.StayFixed;
import frc.robot.commands.MainSystem.TakeAlgaeGround;
import frc.robot.commands.MainSystem.TakeCoral;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final PutCoralStage1 m_putCoralStage1 = new PutCoralStage1(m_elevatorSubsystem, m_armSubsystem);
  private final PutCoralStage2 m_putCoralStage2 = new PutCoralStage2(m_elevatorSubsystem, m_armSubsystem);
  private final PutCoralStage3 m_putCoralStage3 = new PutCoralStage3(m_elevatorSubsystem, m_armSubsystem);
  private final PutCoralStage4 m_putCoralStage4 = new PutCoralStage4(m_elevatorSubsystem, m_armSubsystem);
  private final TakeCoral m_takeCoral  = new TakeCoral(m_elevatorSubsystem, m_armSubsystem);
  private final ShootAlgae m_shootAlgae = new ShootAlgae(m_elevatorSubsystem, m_armSubsystem);
  private final TakeAlgaeGround m_takeAlgaeGround = new TakeAlgaeGround(m_elevatorSubsystem, m_armSubsystem);
  private final StayFixed m_stayFixed = new StayFixed(m_elevatorSubsystem, m_armSubsystem);
  
  private final MainRobotMechanism m_robotMechanism = new MainRobotMechanism();

    // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
    m_driverController;
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

  StructPublisher<Pose3d> vMarker = NetworkTableInstance.getDefault()
      .getStructTopic("Vision/marker0", Pose3d.struct).publish();

  NetworkTableInstance defaultInst = NetworkTableInstance.getDefault();
  NetworkTable vTable = defaultInst.getTable("Vision/Raw0");

  private double eleGeneralHeight = 0;
  private double eleStage0Height = 0;
  private double eleStage1Height = 0;
  private double armJ1Angle = 0;
  private double armJ2Angle = 0;

  private double markerX = 0;
  private double markerY = 0;
  private double markerZ = 0;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private Pose2d robotPose = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
  private double roboAngle = 0;

  public RobotContainer() {
    // Configure the trigger bindings
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
    
    m_operatorController.a().onTrue(m_putCoralStage1);
    m_operatorController.b().onTrue(m_putCoralStage2);
    m_operatorController.x().onTrue(m_putCoralStage3);
    m_operatorController.y().onTrue(m_putCoralStage4);
    m_operatorController.button(5).onTrue(m_takeCoral);
    m_operatorController.button(6).onTrue(m_shootAlgae);
    m_operatorController.button(7).onTrue(m_takeAlgaeGround);
    m_operatorController.button(10).onTrue(m_stayFixed);

    Command driveFieldOrientedDirectAngle      = m_drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);

    m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
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

  public void teleopSimUpdate() {

    eleGeneralHeight = m_elevatorSubsystem.getEncoderDistance();
    eleStage0Height = (eleGeneralHeight > Constants.Elevator.kStage1Height ?  eleGeneralHeight - Constants.Elevator.kStage1Height : 0);
    eleStage1Height = eleGeneralHeight - eleStage0Height;
    armJ1Angle = m_armSubsystem.getSimAngleJ1();
    armJ2Angle = m_armSubsystem.getSimAngleJ2();
    robotPose = m_drivebase.getPose();

    elevatorStage0pub.set(new Pose3d(0,0, eleStage0Height, new Rotation3d(0,0,0)));
    elevatorStage1pub.set(new Pose3d(0,0, eleGeneralHeight, new Rotation3d(0,0,0)));
    armStage0pub.set(new Pose3d(Constants.Arm.FirstJoint.kSimOffsets[0], Constants.Arm.FirstJoint.kSimOffsets[1], Constants.Arm.FirstJoint.kSimOffsets[2] + eleGeneralHeight, 
        new Rotation3d(0,Units.degreesToRadians(armJ1Angle),0)));
    armStage1pub.set(new Pose3d(Constants.Arm.SecondJoint.kSimOffsets[0] + Constants.Arm.FirstJoint.kArmLength * Math.sin(Units.degreesToRadians(armJ1Angle)),
                                Constants.Arm.SecondJoint.kSimOffsets[1], 
                                Constants.Arm.SecondJoint.kSimOffsets[2] + Constants.Arm.FirstJoint.kArmLength * Math.cos(Units.degreesToRadians(armJ1Angle)) + eleGeneralHeight, 
        new Rotation3d(0,Units.degreesToRadians(armJ1Angle+ armJ2Angle-90),0)));
      
    m_robotMechanism.update(eleGeneralHeight, armJ1Angle, armJ2Angle);

    roboAngle = robotPose.getRotation().getRadians() - Math.PI/2;
    
    if (vTable.getEntry("marker_id").getDouble(-1) != -1)
    {
      markerX = vTable.getEntry("x").getDouble(0)*Math.cos(roboAngle)
              - vTable.getEntry("y").getDouble(0)*Math.sin(roboAngle) 
              + robotPose.getTranslation().getX();
      markerY = vTable.getEntry("y").getDouble(0)*Math.cos(roboAngle) 
              + vTable.getEntry("x").getDouble(0)*Math.sin(roboAngle) 
              + robotPose.getTranslation().getY();
      markerZ = vTable.getEntry("z").getDouble(0) + 0.6;

      vMarker.set(new Pose3d(markerX,markerY,markerZ, new Rotation3d(0,0,0)));
    }
    else vMarker.set(new Pose3d(robotPose.getTranslation().getX(),robotPose.getTranslation().getY(),0.6, new Rotation3d(0,0,0)));
  }

}
