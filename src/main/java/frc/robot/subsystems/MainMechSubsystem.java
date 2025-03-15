// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.StatePositions;
import frc.robot.sims.MainRobotMechanism;
import edu.wpi.first.math.util.Units;

public class MainMechSubsystem extends SubsystemBase {
  /** Creates a new MainMechSubsystem. */
  private final ArmSubsystem m_armSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final GripperSubsystem m_gripperSubsystem;
  private final MainRobotMechanism m_mechanism = new MainRobotMechanism();

  
  StructPublisher<Pose3d> elevatorStage0pub = NetworkTableInstance.getDefault()
    .getStructTopic("3dSim/eleStage0", Pose3d.struct).publish();
  StructPublisher<Pose3d> elevatorStage1pub = NetworkTableInstance.getDefault()
    .getStructTopic("3dSim/eleStage1", Pose3d.struct).publish();
  StructPublisher<Pose3d> armStage0pub = NetworkTableInstance.getDefault()
    .getStructTopic("3dSim/armStage0", Pose3d.struct).publish();
  StructPublisher<Pose3d> armStage1pub = NetworkTableInstance.getDefault()
    .getStructTopic("3dSim/armStage1", Pose3d.struct).publish();

  StructPublisher<Pose3d> fakeRobotPose = NetworkTableInstance.getDefault()
    .getStructTopic("3dSim/fakeRobots", Pose3d.struct).publish();

  private double eleGeneralHeight = 0;
  private double eleStage0Height = 0;
  private double eleStage1Height = 0;
  private double armJ1Angle = 0;
  private double armJ2Angle = 0;

  private double armJ1limitCCW = Arm.FirstJoint.kMaxAngle;
  private double armJ1limitCW = Arm.FirstJoint.kMinAngle;
  private double armJ2limitCCW = Arm.SecondJoint.kMaxAngle;
  private double armJ2limitCW = Arm.SecondJoint.kMinAngle;

  private String lastState = "FullyClosed";
  private boolean isGoalReached = false;

  public MainMechSubsystem(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, GripperSubsystem gripperSubsystem) {
    m_armSubsystem = armSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_gripperSubsystem = gripperSubsystem;
  }

  @Override
  public void periodic() {
    eleGeneralHeight = m_elevatorSubsystem.getEncoderDistance();
    armJ1Angle = m_armSubsystem.getFirstJointAngle();
    armJ2Angle = m_armSubsystem.getSecondJointAngle();
    CalculateArmAngleLimits();
    updateMech();
    SmartDashboard.putString("MechState", lastState);
    SmartDashboard.putBoolean("MechGoalReached", isGoalReached);

  }

  public Command CoralIntakeCommand() {
    return run(() -> {MechStateControl("CoralIntake");}).until(this::isGoalReached);
  }

  public Command CoralStage1Command() {
    return run(() -> {MechStateControl("CoralStage1");}).until(this::isGoalReached);
  }

  public Command CoralStage2Command() {
    return run(() -> {MechStateControl("CoralStage2");}).until(this::isGoalReached);
  }

  public Command CoralStage3Command() {
    return run(() -> {MechStateControl("CoralStage3");}).until(this::isGoalReached);
  }

  public Command CoralStage4Command() {
    return run(() -> {MechStateControl("CoralStage4");}).until(this::isGoalReached);
  }

  public Command ThrowAlgaeNetCommand() {
    return run(() -> {MechStateControl("ThrowAlgaeNet");}).until(this::isGoalReached);
  }

  public Command RecoverFromAlgaeCommand() {
    return run(() -> {MechStateControl("RecoverFromAlgae");}).until(this::isGoalReached);
  }

  public Command ThrowAlgaeProcessorCommand() {
    return run(() -> {MechStateControl("ThrowAlgaeProcessor");}).until(this::isGoalReached);
  }

  public Command AlgaeGroundCommand() {
    return run(() -> {MechStateControl("AlgaeGround");}).until(this::isGoalReached);
  }

  public Command Algae23Command() {
    return run(() -> {MechStateControl("Algae23");}).until(this::isGoalReached);
  }

  public Command Algae34Command() {
    return run(() -> {MechStateControl("Algae34");}).until(this::isGoalReached);
  }

  public Command AlgaeFromCoralCommand() {
    return run(() -> {MechStateControl("AlgaeFromCoral");}).until(this::isGoalReached);
  }

  public Command AlgaeCarryCommand() {
    return run(() -> {MechStateControl("AlgaeCarry");}).until(this::isGoalReached);
  }

  public Command FullyClosedCommand() {
    return run(() -> {MechStateControl("FullyClosed");}).until(this::isGoalReached);
  }

  public Command ClosedCommand() {
    return run(() -> {MechStateControl("Closed");}).until(this::isGoalReached);
  }

  public Command CoralCarryCommand()
  {
    return run(() -> {MechStateControl("CoralCarry");}).until(this::isGoalReached);
  }

  public void MechStateControl(String state) {
    if(!RobotState.isAutonomous())
    {
    if(lastState == "FullyClosed" && state != "Closed") state = "FullyClosed";
    else if(lastState != "Closed" && state == "FullyClosed") state = "Closed";
    }
    if(m_gripperSubsystem.hasAlgae())
    {
      if(!java.util.Arrays.asList("ThrowAlgaeNet", "ThrowAlgaeProcessor", "AlgaeGround", "Algae23", "Algae34", "AlgaeFromCoral", "AlgaeCarry").contains(state))
      {
        state = lastState;
      }
    }

    lastState = state;
    switch(state)
    {
      case "CoralIntake":
        CoralIntake();
        break;
      case "CoralCarry":
        CoralCarry();
        break;
      case "CoralStage1":
        CoralStage1();
        break;
      case "CoralStage2":
        CoralStage2();
        break;
      case "CoralStage3":
        CoralStage3();
        break;
      case "CoralStage4":
        CoralStage4();
        break;
      case "ThrowAlgaeNet":
        ThrowAlgaeNet();
        break;
      case "ThrowAlgaeProcessor":
        ThrowAlgaeProcessor();
        break;
      case "AlgaeGround":
        AlgaeGround();
        break;
      case "Algae23":
        Algae23();
        break;
      case "Algae34":
        Algae34();
        break;
      case "AlgaeFromCoral":
        AlgaeFromCoral();
        break;
      case "RecoverFromNet":
        RecoverFromNet();
        break;
      case "AlgaeCarry":
        AlgaeCarry();
        break;
      case "FullyClosed":
        FullyClosed();
        break;
      case "Closed":
        Closed();
        break;
      default:
        Closed();
        break;
    }
  }

  public void reachGoal(double height, double angleJ1, double angleJ2)
  {
    m_elevatorSubsystem.reachGoal(height);
    m_armSubsystem.reachGoal(angleJ1, angleJ2);
    isGoalReached = m_armSubsystem.isGoalReached() && m_elevatorSubsystem.isGoalReached();
  }

  public void RecoverFromNet()
  {

  }

  public void CoralIntake() {
    reachGoal(StatePositions.CoralIntake[0], StatePositions.CoralIntake[1], StatePositions.CoralIntake[2]);
  }

  public void CoralStage1() {
    reachGoal(StatePositions.CoralStage1[0], StatePositions.CoralStage1[1], StatePositions.CoralStage1[2]);
  }

  public void CoralStage2() {
    reachGoal(StatePositions.CoralStage2[0], StatePositions.CoralStage2[1], StatePositions.CoralStage2[2]);
  }

  public void CoralStage3() {
    reachGoal(StatePositions.CoralStage3[0], StatePositions.CoralStage3[1], StatePositions.CoralStage3[2]);
  }

  public void CoralStage4() {
    reachGoal(StatePositions.CoralStage4[0], StatePositions.CoralStage4[1], StatePositions.CoralStage4[2]);
  }

  public void CoralCarry() {
    reachGoal(StatePositions.CoralCarry[0], StatePositions.CoralCarry[1], StatePositions.CoralCarry[2]);
  }

  public void ThrowAlgaeNet() {
    reachGoal(StatePositions.AlgaeThrowNet[0], StatePositions.AlgaeThrowNet[1], StatePositions.AlgaeThrowNet[2]);
  }

  public void ThrowAlgaeProcessor() {
    reachGoal(StatePositions.AlgaeThrowProcessor[0], StatePositions.AlgaeThrowProcessor[1], StatePositions.AlgaeThrowProcessor[2]);
  }

  public void AlgaeGround() {
    reachGoal(StatePositions.AlgaeGround[0], StatePositions.AlgaeGround[1], StatePositions.AlgaeGround[2]);
  }

  public void Algae23() {
    reachGoal(StatePositions.AlgaeStage23[0], StatePositions.AlgaeStage23[1], StatePositions.AlgaeStage23[2]);
  }

  public void Algae34() {
    reachGoal(StatePositions.AlgaeStage34[0], StatePositions.AlgaeStage34[1], StatePositions.AlgaeStage34[2]);
  }

  public void AlgaeFromCoral() {
    reachGoal(StatePositions.AlgaeFromCoral[0], StatePositions.AlgaeFromCoral[1], StatePositions.AlgaeFromCoral[2]);
  }

  public void AlgaeCarry() {
    reachGoal(StatePositions.AlgaeCarry[0], StatePositions.AlgaeCarry[1], StatePositions.AlgaeCarry[2]);
  }

  public void Closed() {
    reachGoal(StatePositions.Closed[0], StatePositions.Closed[1], StatePositions.Closed[2]);
  }

  public void FullyClosed() {
    reachGoal(StatePositions.FullyClosed[0], StatePositions.FullyClosed[1], StatePositions.FullyClosed[2]);
  }

  public boolean isGoalReached()
  {
    return isGoalReached;
  }

  public void CalculateArmAngleLimits()
  {
    double eleArmHeight = eleGeneralHeight + 0.1;
    if(eleArmHeight/Arm.FirstJoint.kArmLength < 1)
    {
      armJ1limitCW = Units.radiansToDegrees(Math.acos(eleArmHeight/(Arm.FirstJoint.kArmLength*Arm.FirstJoint.kArmSafetyFactor)));
      armJ1limitCCW = 360 - armJ1limitCW;
    }
    else
    {
      armJ1limitCW = Arm.FirstJoint.kMinAngle;
      armJ1limitCCW = Arm.FirstJoint.kMaxAngle;
    }

    Double j2Height = eleArmHeight - Arm.FirstJoint.kArmLength*Arm.FirstJoint.kArmSafetyFactor*Math.cos(Units.degreesToRadians(armJ1Angle));

    if(j2Height/Arm.SecondJoint.kArmLength < 1)
    { 
      if(armJ1Angle < 180)
      {
        armJ2limitCW = 180 - armJ1Angle + (Units.radiansToDegrees(Math.acos(j2Height/(Arm.SecondJoint.kArmLength*Arm.SecondJoint.kArmSafetyFactor))));
        armJ2limitCCW = 360 - armJ2limitCW;
      }
      else
      {
        armJ2limitCCW = 495 - armJ1Angle - (Units.radiansToDegrees(Math.acos(j2Height/(Arm.SecondJoint.kArmLength*Arm.SecondJoint.kArmSafetyFactor))));
        armJ2limitCW = 30;
      }
    }
    else
    {
      armJ2limitCW = Arm.SecondJoint.kMinAngle;
      armJ2limitCCW = Arm.SecondJoint.kMaxAngle;
    }

    SmartDashboard.putNumber("Arm/J1-CW-Limit", armJ1limitCW);
    SmartDashboard.putNumber("Arm/J1-CCW-Limit", armJ1limitCCW);
    SmartDashboard.putNumber("Arm/J2-CW-Limit", armJ2limitCW);
    SmartDashboard.putNumber("Arm/J2-CCW-Limit", armJ2limitCCW);
  }

  public void updateMech()
  {
    eleStage0Height = (eleGeneralHeight > Constants.Elevator.kStage1Height ?  eleGeneralHeight - Constants.Elevator.kStage1Height : 0);
    eleStage1Height = eleGeneralHeight - eleStage0Height;
    armJ1Angle = m_armSubsystem.getFirstJointAngle();
    armJ2Angle = m_armSubsystem.getSecondJointAngle();

    elevatorStage0pub.set(new Pose3d(0,0, eleStage0Height, new Rotation3d(0,0,0)));

    elevatorStage1pub.set(new Pose3d(0,0, eleGeneralHeight, new Rotation3d(0,0,0)));

    armStage0pub.set(new Pose3d(Constants.Arm.FirstJoint.kSimOffsets[0], Constants.Arm.FirstJoint.kSimOffsets[1], Constants.Arm.FirstJoint.kSimOffsets[2] + eleGeneralHeight, 
        new Rotation3d(0, Units.degreesToRadians(180 - armJ1Angle),0)));
    armStage1pub.set(new Pose3d(Constants.Arm.SecondJoint.kSimOffsets[0] + Constants.Arm.FirstJoint.kArmLength * Math.sin(Units.degreesToRadians(180 - armJ1Angle)),
                                Constants.Arm.SecondJoint.kSimOffsets[1], 
                                Constants.Arm.SecondJoint.kSimOffsets[2] + Constants.Arm.FirstJoint.kArmLength * Math.cos(Units.degreesToRadians(180 - armJ1Angle)) + eleGeneralHeight, 
        new Rotation3d(0, Units.degreesToRadians((180 - armJ1Angle) + (180 - armJ2Angle) -90),0)));

    m_mechanism.update(0, armJ1Angle, armJ2Angle);
    
    fakeRobotPose.set(new Pose3d(2,2,0, new Rotation3d()));
  }

  public Command setMotorPositions()
  {
    return run(() -> resetMechanisms()).until(() -> m_armSubsystem.checkMotorsSet());
  }

  public void resetMechanisms() {
    m_armSubsystem.resetArmPositions();
    m_elevatorSubsystem.resetMotorPosition();
  }

}
