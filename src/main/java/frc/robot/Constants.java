// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }
  public static class ElevatorConstants {
    public static final int DEVICE_ID = 10;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
  }
  public static class ArmConstants {
    public static final int SHOULDER_DEVICE_ID = 12;
    public static final int ELBOW_DEVICE_ID = 13;

    public static final int SHOULDER_ENCODER_ID = 0;
    public static final double SHOULDER_ENCODER_INIT = 0;

    public static final int ELBOW_ENCODER_ID = 1;
    public static final double ELBOW_ENCODER_INIT = 0;

    public static final double kPS = 0;
    public static final double kIS = 0;
    public static final double kDS = 0;
    public static final double kSS = 0;
    public static final double kVS = 0;

    public static final double kPE = 0;
    public static final double kIE = 0;
    public static final double kDE = 0;
    public static final double kSE = 0;
    public static final double kVE = 0;
  }
  public static class GripperConstants {
    public static final int VORTEX_SPARK_MAX_ID = 34;
    
    public static final double POSITION_CONVERSION_FACTOR = 0;
    public static final double VELOCITY_CONVERSION_FACTOR = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;

    public static final double MAX_ACC = 0;
    public static final double MAX_VEL = 0;
    public static final double ALLOWED_ERR = 0;
    public static final double LOW_OUT = 0;
    public static final double HIGH_OUT = 0;
  }
}
