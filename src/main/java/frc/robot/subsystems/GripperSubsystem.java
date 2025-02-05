// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Unused for now. DO NOT COMMIT.
 */
public class GripperSubsystem extends SubsystemBase {
  //private final SparkMax m_vortex = new SparkMax(34, MotorType.kBrushless);
  
  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {

  }

  //public Command controlWithTriggers(double input) {return run(() -> m_vortex.set(input/2));}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
