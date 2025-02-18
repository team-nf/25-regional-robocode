package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ElevatorConstants;

/** Helper class for creating a {@link Mechanism2d} and 3D component representation of an arm. */
public class ArmVisualizer {
  private final String logKey;
    
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismRoot2d mechanismRoot;
  private final LoggedMechanismLigament2d elevatorLigament;
  private final LoggedMechanismLigament2d shoulderLigament;
  private final LoggedMechanismLigament2d elbowLigament;
  private final LoggedMechanismLigament2d wristLigament;
  

  public ArmVisualizer(String logKey, Color8Bit colorOverride) {
    this.logKey = logKey;
    mechanism = new LoggedMechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("Arm", 2, 0);
    elevatorLigament =
        mechanismRoot.append(
        new LoggedMechanismLigament2d(
            "Elevator", ElevatorConstants.MIN_HEIGHT.magnitude(), 90, 6, new Color8Bit(Color.kBlack)));
    shoulderLigament =
        elevatorLigament.append(
            new LoggedMechanismLigament2d(
                "Shoulder",
                0.345,
                -90,
                4,
                colorOverride != null ? colorOverride : new Color8Bit(Color.kDarkBlue)));
    elbowLigament =
        shoulderLigament.append(
            new LoggedMechanismLigament2d(
                "Elbow",
                0.345,
                -90,
                4,
                colorOverride != null ? colorOverride : new Color8Bit(Color.kBlue)));
    wristLigament =
        elbowLigament.append(
            new LoggedMechanismLigament2d(
                "Wrist",
                0.105,
                0,
                4,
                colorOverride != null ? colorOverride : new Color8Bit(Color.kSkyBlue)));
  }

    public void update(double elevatorLength) {
    elevatorLigament.setLength(elevatorLength);
    Logger.recordOutput("Mechanism2d/" + logKey, mechanism);

    var elevatorPose =
            new Pose3d(0, 0, elevatorLength, new Rotation3d());
        var shoulderPose =
            new Pose3d(
                0,
                0.0,
                0.5,
                new Rotation3d(0.0, -shoulderLigament.getAngle(), 0.0));
        var elbowPose =
            shoulderPose.transformBy(
                new Transform3d(
                    new Translation3d(0.345, 0.0, 0.0),
                    new Rotation3d(0.0, -elbowLigament.getAngle(), 0.0)));
        var wristPose =
            elbowPose.transformBy(
                new Transform3d(
                    new Translation3d(0.345, 0.0, 0.0),
                    new Rotation3d(0.0, -wristLigament.getAngle(), 0.0)));
        Logger.recordOutput("Mechanism3d/" + logKey, elevatorPose, shoulderPose, elbowPose, wristPose);
    }

    public void update(double shoulderAngle, double elbowAngle) {
        shoulderLigament.setAngle(Units.radiansToDegrees(shoulderAngle) - 90.0);
        elbowLigament.setAngle(Units.radiansToDegrees(elbowAngle));
        Logger.recordOutput("Mechanism2d/" + logKey, mechanism);
    
        var elevatorPose =
            new Pose3d(0, 0, elevatorLigament.getLength(), new Rotation3d());
        var shoulderPose =
            new Pose3d(
                0,
                0.0,
                0.5,
                new Rotation3d(0.0, -shoulderAngle, 0.0));
        var elbowPose =
            shoulderPose.transformBy(
                new Transform3d(
                    new Translation3d(0.345, 0.0, 0.0),
                    new Rotation3d(0.0, -elbowAngle, 0.0)));
        var wristPose =
            elbowPose.transformBy(
                new Transform3d(
                    new Translation3d(0.345, 0.0, 0.0),
                    new Rotation3d(0.0, -wristLigament.getAngle(), 0.0)));
        Logger.recordOutput("Mechanism3d/" + logKey, elevatorPose, shoulderPose, elbowPose, wristPose);
      }

    public void update(double shoulderAngle, double elbowAngle, double wristAngle) {
    shoulderLigament.setAngle(Units.radiansToDegrees(shoulderAngle) - 90.0);
    elbowLigament.setAngle(Units.radiansToDegrees(elbowAngle));
    wristLigament.setAngle(Units.radiansToDegrees(wristAngle));
    Logger.recordOutput("Mechanism2d/" + logKey, mechanism);

    var elevatorPose =
        new Pose3d(0, 0, elevatorLigament.getLength(), new Rotation3d());
    var shoulderPose =
        new Pose3d(
            0,
            0.0,
            0.5,
            new Rotation3d(0.0, -shoulderAngle, 0.0));
    var elbowPose =
        shoulderPose.transformBy(
            new Transform3d(
                new Translation3d(0.345, 0.0, 0.0),
                new Rotation3d(0.0, -elbowAngle, 0.0)));
    var wristPose =
        elbowPose.transformBy(
            new Transform3d(
                new Translation3d(0.345, 0.0, 0.0),
                new Rotation3d(0.0, -wristAngle, 0.0)));
    Logger.recordOutput("Mechanism3d/" + logKey, elevatorPose, shoulderPose, elbowPose, wristPose);
  }

  public LoggedMechanism2d getMechanism2d() {
    return this.mechanism;
  }
}