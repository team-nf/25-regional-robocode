// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double MAX_SPEED = 4.0;

  public class Swerve
  {

    public static final double MaxSpeed = 0;
    public static final double MaxAngularRate = 0;

  }

  public class InitialConstants {
    public static final double[] EncoderStartAngles = {264.4/2,8.7/2};
  }

  public static class TestingConstants {
    public static final boolean kTestWithSpinning = false;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.1;
  }

  public static class GripperConstants {
    public static final int kGripperID = 51;
    public static final double kGripper_kP = 0.11;
    public static final double kGripper_kI = 0.0;
    public static final double kGripper_kD = 0.0;
    public static final double kGripper_kS = 0.1;
    public static final double kGripper_kV = 0.12;
    public static final double kGripper_kPFV = 8.0;

    public static final int kAlgaeSensor = 9;
    public static final int kCoralSensor = 8;
  }

  public class Arm {

    public class FirstJoint {
      public static final int kMotorPort = 61;
      public static final int kEncoderChannel = 0;

      public static final String kArmPositionKey = "ArmPosition_J1";
      public static final String kArmPKey = "ArmP_J1";

      // The P gain for the PID controller that drives this arm.
      public static final double kArmJoint1_kP = 0.7;
      public static final double kArmJoint1_kI = 0.0;
      public static final double kArmJoint1_kD = 0.1;
      public static final double kArmJoint1_kS = 0.0;
      public static final double kArmJoint1_kV = 0.0;
      public static final double kArmJoint1_kA = 0.0;
      public static final double kArmJoint1_kG = 0.05;
      public static final double kArmJoint1_kPFV = 4;
      public static final double kArmJoint1_kSCL = 40;
      public static final double kArmJoint1_kSCLL = 15;

      public static final double kArmJoint1_MMCV = 200; // Cruise Velocity
      public static final double kArmJoint1_MMA = 200; // Acceleration
      public static final double kArmJoint1_MMJ = 0; // Jerk

      public static final double kDefaultArmSetpointDegrees = 75.0;

      // distance per pulse = (angle per revolution) / (pulses per revolution)
      //  = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kArmReduction = 172.770;
      public static final double kArmMass = 6.5; // Kilograms
      public static final double kArmLength = 0.350;

      public static final double kMinAngle = 0;
      public static final double kMaxAngle = 360;

      public static final double kMinAngleRads = Units.degreesToRadians(-180);        // For sim
      public static final double kMaxAngleRads = Units.degreesToRadians(180); // For sim

      public static final double[] kSimOffsets = {0.091,0.056,0.275};

      public static final double kAngleTolerance = 0.5;
      public static final double kArmSafetyFactor = 1.5;

    }

   

    public class SecondJoint {
      public static final int kMotorPort = 62;
      public static final int kEncoderChannel = 1;
  

      public static final String kArmPositionKey = "ArmPosition_J2";
      public static final String kArmPKey = "ArmP_J2";

      // The P gain for the PID controller that drives this arm.
      public static final double kDefaultArmSetpointDegrees = 75.0;

      // The P gain for the PID controller that drives this arm.
      public static final double kArmJoint2_kP = 0.6;
      public static final double kArmJoint2_kI = 0.03;
      public static final double kArmJoint2_kD = 0.1;
      public static final double kArmJoint2_kS = 0.0;
      public static final double kArmJoint2_kV = 0.0;
      public static final double kArmJoint2_kA = 0.0;
      public static final double kArmJoint2_kG = 0.05;
      public static final double kArmJoint2_kPFV = 4;
      public static final double kArmJoint2_kSCL = 40;
      public static final double kArmJoint2_kSCLL = 15;

      public static final int kArmJoint2_MMCV = 200; // Cruise Velocity
      public static final int kArmJoint2_MMA = 140; // Acceleration
      public static final int kArmJoint2_MMJ = 0; // Jerk
      
      // distance per pulse = (angle per revolution) / (pulses per revolution)
      //  = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kArmReduction = 112.92;
      public static final double kPulleyErrorRatio = 0.4;
      public static final double kArmMass = 4.0; // Kilograms
      public static final double kArmLength = 0.350;
      public static final double kMinAngle = 0;
      public static final double kMaxAngle = 360;

      public static final double kMinAngleRads = Units.degreesToRadians(-180);        // For sim
      public static final double kMaxAngleRads = Units.degreesToRadians(180); // For sim

      public static final double[] kSimOffsets = {0.091,0.007,0.275};
      
      public static final double kAngleTolerance = 0.5;
      public static final double kArmSafetyFactor = 1;
    }

  }
   

  public static class Elevator {
    public static final int kMotorPort = 10;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;

    public static final double kStage1Height = 0.65;
/** Benim denerken son ulaştığım değerleri geçiriyorum
    public static final double kElevatorKp = 0.5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0.04;
    */
    public static final double kElevatorKp = 0.625;
    public static final double kElevatorKi = 0.04;
    public static final double kElevatorKd = 0.01;

    public static final double kElevatorkS = 0.0; // volts (V)
    //public static final double kElevatorkG = 0.8; // volts (V)
    public static final double kElevatorkG = -0.07; // ters yöne güç verince asansör yer çekimine karşı hareket ediyor değiştirmediysen bu kodda
    public static final double kElevatorkV = 0.0; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorMMCV = 200; //12V sınır koyunca maximum 90 cıvarına çıktı zaten velocity
    //+bence iki kat daha hızlı olabilir ama mümkün görünmüyor iyi bir sınır olmalı
    public static final double kElevatorMMA = 160;
    public static final double kElevatorMMJ = 0; // kullanmıyoruz

    public static final double kElevatorGearing = 12.0;
    public static final double kElevatorDrumRadius = 0.021;
    public static final double kElevatorDistPerRotation = 0.02;
    public static final double kCarriageMass = 13.0; // kg

    public static final double kAmpLimit = 40.0;
    public static final double kVoltageLimit = 9.0;

    public static final double kSetpointMeters = 0.75;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.07;
    public static final double kMaxElevatorHeightMeters = 1.45;
    public static final double kElevatorTolerance = 0.06;
    public static final double kReadyPos = 0.2;
    public static final double kHeightTolerance = 0.02;
  }

  public class StatePositions
  {
    // Length, Angle 1, Angle 2
    public static final double[] CoralStage1 = {0.25, 150, 170};      //Coral Intake
    public static final double[] CoralStage2 = {0.25, 160, 168};   //Coral Stage 1
    public static final double[] CoralStage3 = {0.51, 180, 145};    //Coral Stage 2
    public static final double[] CoralStage4 = {1.21, 180, 145};    //Coral Stage 3
    public static final double[] CoralIntake = {0.38, 155, 340};    //Coral Stage 4
    public static final double[] AlgaeThrowNet = {1.40, 187, 190};    //Algae Shoot
    public static final double[] AlgaeThrowProcessor = {1.3, 187, 190};    //Algae Shoot
    public static final double[] AlgaeStage23 = {0.72, 250, 150};     //Algae Stage 2-3
    public static final double[] AlgaeStage34 = {0.28, 250, 150};     //Algae Stage 3-4
    public static final double[] AlgaeGround = {0.07, 270, 196};  //Algae Ground
    public static final double[] AlgaeFromCoral = {0.25, 270, 180};  //Algae Ground
    public static final double[] AlgaeCarry = {0.3, 180, 180};  //Algae Ground
    public static final double[] Closed = {0.15, 170, 340};  //Closed
    public static final double[] FullyClosed = {0.0, 170, 340};
  }

  public class AutoConstants
  {
    public static final double[] stage4Pos18R = {2.96,3.93,0};
    public static final double[] stage3Pos18R = {2.96,3.93,0};
    public static final double[] stage4Pos18L = {2.98,4.28,0};
    public static final double[] stage3Pos18L = {2.98,4.28,0};
    public static final double[] checkTagPos18 = {2.5,4,0};

    public static final double[] tag1 = {16.6972, 0.6553, 1.4859, 126, 0};
    public static final double[] tag2 = {16.6972, 7.3965, 1.4859, 234, 0};
    public static final double[] tag3 = {11.5608, 8.0556, 1.3017, 270, 0};
    public static final double[] tag4 = {9.2761, 6.1377, 1.8679, 0, 30};
    public static final double[] tag5 = {9.2761, 1.9149, 1.8679, 0, 30};
    public static final double[] tag6 = {13.4744, 3.3063, 0.3081, 300, 0};
    public static final double[] tag7 = {13.8905, 4.0259, 0.3081, 0, 0};
    public static final double[] tag8 = {13.4744, 4.7455, 0.3081, 60, 0};
    public static final double[] tag9 = {12.6434, 4.7455, 0.3081, 120, 0};
    public static final double[] tag10 = {12.2273, 4.0259, 0.3081, 180, 0};
    public static final double[] tag11 = {12.6434, 3.3063, 0.3081, 240, 0};
    public static final double[] tag12 = {0.8512, 0.6553, 1.4859, 54, 0};
    public static final double[] tag13 = {0.8512, 7.3965, 1.4859, 306, 0};
    public static final double[] tag14 = {8.2723, 6.1377, 1.8679, 180, 30};
    public static final double[] tag15 = {8.2723, 1.9149, 1.8679, 180, 30};
    public static final double[] tag16 = {5.9875, -0.0038, 1.3017, 90, 0};
    public static final double[] tag17 = {4.0739, 3.3063, 0.3081, 240, 0};
    public static final double[] tag18 = {3.6576, 4.0259, 0.3081, 180, 0};
    public static final double[] tag19 = {4.0739, 4.7455, 0.3081, 120, 0};
    public static final double[] tag20 = {4.9047, 4.7455, 0.3081, 60, 0};
    public static final double[] tag21 = {5.3210, 4.0259, 0.3081, 0, 0};
    public static final double[] tag22 = {4.9047, 3.3063, 0.3081, 300, 0};

    public static final double xOffsetS4R = stage4Pos18R[0] - tag18[0];
    public static final double yOffsetS4R = stage4Pos18R[1] - tag18[1];
    public static final double zRotOffsetS4R = stage4Pos18R[2] - tag18[3];

    public static final double xOffsetS3R = stage3Pos18R[0] - tag18[0];
    public static final double yOffsetS3R = stage3Pos18R[1] - tag18[1];
    public static final double zRotOffsetS3R = stage3Pos18R[2] - tag18[3];

    public static final double xOffsetS4L = stage4Pos18L[0] - tag18[0];
    public static final double yOffsetS4L = stage4Pos18L[1] - tag18[1];
    public static final double zRotOffsetS4L = stage4Pos18L[2] - tag18[3];

    public static final double xOffsetS3L = stage3Pos18L[0] - tag18[0];
    public static final double yOffsetS3L = stage3Pos18L[1] - tag18[1];
    public static final double zRotOffsetS3L = stage3Pos18L[2] - tag18[3];

    public static final double xOffsetCT = checkTagPos18[0] - tag18[0];
    public static final double yOffsetCT = checkTagPos18[1] - tag18[1];
    public static final double zRotOffsetCT = checkTagPos18[2] - tag18[3];

    public static final Pose2d TagToRobot = new Pose2d(-xOffsetCT, -yOffsetCT, new Rotation2d(-zRotOffsetCT));
    public static final Pose2d TagToReefPosS4R = new Pose2d(-xOffsetS4R, -yOffsetS4R, new Rotation2d(-zRotOffsetS4R));
    public static final Pose2d TagToReefPosS3R = new Pose2d(-xOffsetS3R, -yOffsetS3R, new Rotation2d(-zRotOffsetS3R));
    public static final Pose2d TagToReefPosS4L = new Pose2d(-xOffsetS4L, -yOffsetS4L, new Rotation2d(-zRotOffsetS4L));
    public static final Pose2d TagToReefPosS3L = new Pose2d(-xOffsetS3L, -yOffsetS3L, new Rotation2d(-zRotOffsetS3L));

    public static final double LL_Accuracy_mt1 = 0.025;
  }

}
