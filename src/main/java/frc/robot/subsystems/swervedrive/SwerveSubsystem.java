// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.Meter;


public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve/teamnf");

  StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("3dSim/fakeRobot", Pose3d.struct).publish();

  private double driveMultiplier = 1;


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.INFO;
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    //swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    //swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    setupPathPlanner();
  }

  @Override
  public void periodic() {
    if (Robot.isReal())
    {
      updateOdometryWithLL_mt1();
      publisher.set(new Pose3d(swerveDrive.getPose().getX(),swerveDrive.getPose().getY(),0, new Rotation3d(swerveDrive.getPose().getRotation())));
    }

    if(java.util.Arrays.asList("ThrowAlgaeNet", "AlgaeGround", "Algae23", "Algae34", "CoralStage4", "CoralStage3", "CoralStage2")
              .contains(SmartDashboard.getString("MechState", "Closed"))) driveMultiplier = 0.5;
    else driveMultiplier = 1;
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
  
  public void updateOdometryWithLL_mt1() {
    boolean doRejectUpdate = false;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
    if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
    {
      if(mt1.rawFiducials[0].ambiguity > .7)
      {
        doRejectUpdate = true;
      }
      if(mt1.rawFiducials[0].distToCamera > 3)
      {
        doRejectUpdate = true;
      }
      if(mt1.rawFiducials[0].ta < AutoConstants.LL_Accuracy) doRejectUpdate = true;
      SmartDashboard.putNumber("Vision_ta", mt1.rawFiducials[0].ta);

    }

    if(mt1.tagCount == 0)
    {
      doRejectUpdate = true;
    }

    if(!doRejectUpdate)
    {
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
      swerveDrive.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds);
    }


    swerveDrive.updateOdometry();
  }

  public void updateOdometryWithLL_mt2() {
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation("limelight", swerveDrive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(Math.abs(swerveDrive.getGyro().getYawAngularVelocity().magnitude()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0)
    {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      swerveDrive.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    }

    swerveDrive.updateOdometry();
  }

  public PathConstraints getConstraints() {
    return new PathConstraints(
        swerveDrive.getMaximumChassisVelocity()/1.5, 1.0/1.5,
        swerveDrive.getMaximumChassisAngularVelocity()/1.5, Units.degreesToRadians(180)/1.5);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public Command goToReef(int id, boolean isLeft, int stage)
  {
    double x_offset = AutoConstants.xOffsetS4;
    double y_offset = AutoConstants.yOffsetS4;
    double theta_offset = AutoConstants.zRotOffsetS4;

    if(stage == 4)
    {
      x_offset = AutoConstants.xOffsetS4;
      y_offset = AutoConstants.yOffsetS4;
      theta_offset = AutoConstants.zRotOffsetS4;
    }
    else if(stage == 3)
    {
        x_offset = AutoConstants.xOffsetS3;
        y_offset = AutoConstants.yOffsetS3;
        theta_offset = AutoConstants.zRotOffsetS3;
    }

    if(isLeft)
    {
      //x_offset = x_offset;
      y_offset = -y_offset*1.5;
      theta_offset = -theta_offset;
    }

    Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
    
    double theta_tag =  aprilTagPose.getRotation().getZ() + Units.degreesToRadians(AutoConstants.zRotOffsetCT); 

    double x_tag = aprilTagPose.getX() + AutoConstants.xOffsetCT*Math.cos(theta_tag) -  AutoConstants.yOffsetCT*Math.sin(theta_tag);
    double y_tag = aprilTagPose.getY() + AutoConstants.yOffsetCT*Math.cos(theta_tag) +  AutoConstants.xOffsetCT*Math.sin(theta_tag);

    double theta_reef = aprilTagPose.getRotation().getZ() + Units.degreesToRadians(theta_offset);

    double x_reef = aprilTagPose.getX() + x_offset*Math.cos(theta_reef)*0.9 - y_offset*Math.sin(theta_reef)*0.9;
    double y_reef = aprilTagPose.getY() + y_offset*Math.cos(theta_reef)*0.9 + x_offset*Math.sin(theta_reef)*0.9;

    return AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(x_tag,y_tag), new Rotation2d(theta_tag)), getConstraints())
                        .andThen(AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(x_reef,y_reef), new Rotation2d(theta_reef)), getConstraints()));
  }

  public double getDriveMultiplier()
  {
    return driveMultiplier;
  }
}

