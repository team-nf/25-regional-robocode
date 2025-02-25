// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public class Arm {

    public class FirstJoint {
      public static final int kMotorPort = 61;
      public static final int kEncoderChannel = 2;

      public static final String kArmPositionKey = "ArmPosition_J1";
      public static final String kArmPKey = "ArmP_J1";

      // The P gain for the PID controller that drives this arm.
      public static final double kArmJoint1_kP = 1.5;
      public static final double kArmJoint1_kI = 0.0;
      public static final double kArmJoint1_kD = 0.0;
      public static final double kArmJoint1_kS = 0.0;
      public static final double kArmJoint1_kV = 0.0;
      public static final double kArmJoint1_kA = 0.0;
      public static final double kArmJoint1_kG = 0.1;
      public static final double kArmJoint1_kPFV = 8;
      public static final double kArmJoint1_kSCL = 40;
      public static final double kArmJoint1_kSCLL = 15;

      public static final double kDefaultArmSetpointDegrees = 75.0;

      // distance per pulse = (angle per revolution) / (pulses per revolution)
      //  = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kArmReduction = 173;
      public static final double kArmMass = 6.5; // Kilograms
      public static final double kArmLength = 0.350;
      public static final double kMinAngleRads = Units.degreesToRadians(-180);
      public static final double kMaxAngleRads = Units.degreesToRadians(180);

      public static final double[] kSimOffsets = {0.091,0.056,0.275};

      public static double kAngleTolerance = 0.5;
    }

    public class SecondJoint {
      public static final int kMotorPort = 62;
      public static final int kEncoderChannel = 3;

      public static final String kArmPositionKey = "ArmPosition_J2";
      public static final String kArmPKey = "ArmP_J2";

      // The P gain for the PID controller that drives this arm.
      public static final double kDefaultArmSetpointDegrees = 75.0;

      // The P gain for the PID controller that drives this arm.
      public static final double kArmJoint2_kP = 1.5;
      public static final double kArmJoint2_kI = 0.0;
      public static final double kArmJoint2_kD = 0.0;
      public static final double kArmJoint2_kS = 0.0;
      public static final double kArmJoint2_kV = 0.0;
      public static final double kArmJoint2_kA = 0.0;
      public static final double kArmJoint2_kG = 0.1;
      public static final double kArmJoint2_kPFV = 8;
      public static final double kArmJoint2_kSCL = 40;
      public static final double kArmJoint2_kSCLL = 15;
      
      // distance per pulse = (angle per revolution) / (pulses per revolution)
      //  = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kArmReduction = 97;
      public static final double kArmMass = 4.0; // Kilograms
      public static final double kArmLength = 0.350;
      public static final double kMinAngleRads = Units.degreesToRadians(-180);
      public static final double kMaxAngleRads = Units.degreesToRadians(180);

      public static final double[] kSimOffsets = {0.091,0.007,0.275};
      
      public static double kAngleTolerance = 0.5;
    }

  }
   

  public static class Elevator {
    public static final int kMotorPort = 41;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;

    public static final double kStage1Height = 0.65;

    public static final double kElevatorKp = 0.5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0.04;

    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.8; // volts (V)
    public static final double kElevatorkV = 0.0; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing = 12.0;
    public static final double kElevatorDrumRadius = 0.02;
    public static final double kCarriageMass = 13.0; // kg

    public static final double kAmpLimit = 40.0;
    public static final double kVoltageLimit = 9.0;

    public static final double kSetpointMeters = 0.75;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.5;
    public static double kElevatorTolerance = 0.06;
  }

  public class StatePositions
  {
    // Length, Angle 1, Angle 2
    public static final double[] kCoralStage1 = {0, 30, 10};      //Coral Intake
    public static final double[] kCoralStage2 = {0.09, 20, 12};   //Coral Stage 1
    public static final double[] kCoralStage3 = {0.49, 0, 28};    //Coral Stage 2
    public static final double[] kCoralStage4 = {1.3, 5, 35};    //Coral Stage 3
    public static final double[] kCoralIntake = {0.0, 0, -90};    //Coral Stage 4
    public static final double[] kAlgaeThrow = {1.5, -7, -10};    //Algae Shoot
    public static final double[] kAlgaeStage23 = {0.0, -0, 0};     //Algae Stage 2-3
    public static final double[] kAlgaeStage34 = {0.0, -0, 0};     //Algae Stage 3-4
    public static final double[] kAlgaeGround = {0.05, -90, 0};  //Algae Ground
    public static final double[] kClosed = {0.0, -10, -160};  //Closed

  }

}
