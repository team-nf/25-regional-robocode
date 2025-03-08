// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;


public class GripperSubsystem extends SubsystemBase {
 private final SparkMax m_spark = new SparkMax(GripperConstants.kGripperID, MotorType.kBrushless);
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController sparkPID;
  private boolean hasAlgae = false;
  private boolean hasCoral = false;

  private int timerA = 0;
  private int timerC = 0;

  private final DigitalInput m_AlgaeSensor = new DigitalInput(GripperConstants.kAlgaeSensor);
  private final DigitalInput m_coralSensor = new DigitalInput(GripperConstants.kCoralSensor);

  /** 
   * Creates a new GripperSubsystem. 
   * Valla tuna ercan senden kopyaladım.
   */
  public GripperSubsystem() {
      // Create SparkMAX Config Object (I hate this new abundant API.)
      motorConfig = new SparkMaxConfig();
  
      /** 
       * Configure the encoder. We are using the encoder of NEO Vortex, therefore
       * no config is needed, yet adjusting conversion factors is needed. 
       */
      motorConfig.encoder
        //.positionConversionFactor(GripperConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(7.2);
      
      /**
       * Configure the closed loop controller. The feedback sensor is the primary encoder.
       * The closed loop controller will be used for velocity control.
       */
      motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(GripperConstants.kGripper_kP)
        .i(GripperConstants.kGripper_kI)
        .d(GripperConstants.kGripper_kD)
        .velocityFF(GripperConstants.kGripper_kV);
        //.outputRange(GripperConstants.LOW_OUT, GripperConstants.HIGH_OUT);
  
      motorConfig.closedLoop.maxMotion
      // Set MAXMotion parameters for velocity control
        .maxAcceleration(200)
        .maxVelocity(300)
        .allowedClosedLoopError(5);

      motorConfig.smartCurrentLimit(39);

      motorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
  
      // Apply the configuration to the Spark MAX
      m_spark.configure(motorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      sparkPID = m_spark.getClosedLoopController();

  
    // Initialize telemetry
    SmartDashboard.setDefaultNumber("Target Velocity", 0);

  }

  public boolean hasAlgae() {return this.hasAlgae;}

  public boolean hasCoral() {return this.hasCoral;}

  /** Used for testing */
  //public Command controlWithTriggers(double input) {return run(() -> );}

  public Command controlFromDashboard() {
    double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
    return run(() -> sparkPID.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl));
  }

  // Maybe use motion magic?? Test first.

  public Command controlFromTargetVelocity(double targetVelocity) 
  {return run(() -> sparkPID.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl));}

  //public Command takeAlgae() {return runEnd(() -> sparkPID.setReference(.6, ControlType.kMAXMotionVelocityControl), this::stop).until(this::hasAlgae);}
  public Command takeAlgae() {return runEnd(() -> m_spark.set(.6), this::stop).until(this::hasAlgae);}

  //public Command takeCoral() {return runEnd(() -> sparkPID.setReference(-0.3, ControlType.kMAXMotionVelocityControl), this::stop).until(this::hasCoral);}
  public Command takeCoral() {return runEnd(() -> m_spark.set(-0.5), this::stop).until(this::hasCoral);}

  //public Command throwAlgae() {return runEnd(() -> sparkPID.setReference(-0.6, ControlType.kMAXMotionVelocityControl), this::stop);}
  public Command throwAlgae() {return runEnd(() -> m_spark.set(-0.6), this::stop).onlyWhile(this::hasAlgae);}

  //public Command throwCoral() {return runEnd(() -> sparkPID.setReference(0.5, ControlType.kMAXMotionVelocityControl), this::stop);}
  public Command throwCoral() {return runEnd(() -> m_spark.set(.5), this::stop).onlyWhile(this::hasCoral);}
  //public Command stop() {return run(() -> sparkPID.setReference(0.03 ControlType.kMAXMotionVelocityControl));}
  public void stop() {m_spark.stopMotor();}

  public Command stopCommand() {return run((() -> m_spark.stopMotor()));}


  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    // iğrenç şeyler yaptım -yüşa
    
    if(!m_AlgaeSensor.get()) {
      timerA++;
      if (!m_AlgaeSensor.get() && timerA == 50) // periodic 20msde bir çağrılıyor, 1 saniye beklemek için 50 çağrı yapılmalı
      {
        this.hasAlgae = true;
        timerA = 0;
      };
    } else {
      if (hasAlgae) this.hasAlgae = false;
      if (timerA != 0) timerA = 0;
    }

    if(!m_coralSensor.get()) {
      timerC++;
      if (!m_coralSensor.get() && timerC == 50) 
      {
        this.hasCoral = true;
        timerC = 0;
      };
    } else {
      if (hasCoral) this.hasCoral = false;
      if (timerC != 0) timerC = 0;
    }
    
    // Telemetry
    SmartDashboard.putNumber("Current Velocity", m_spark.getAbsoluteEncoder().getVelocity());

    SmartDashboard.putBoolean("Has Coral?: ", this.hasCoral);
    SmartDashboard.putBoolean("Has Algae?: ", hasAlgae);
  }
}