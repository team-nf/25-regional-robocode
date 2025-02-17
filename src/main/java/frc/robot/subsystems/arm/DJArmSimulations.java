package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

/**
 * Helper class for initiating simulation classes of the arm.
 * Wanted to avoid abundant code in the subsystem class.
 * Might move this under {@link ArmVisualizer} instead of having two helper classes.
 * (Does this count a helper class at this point) (idk??)
 */
public class DJArmSimulations {
    private final DCMotor j1Motor;
    private final DCMotor j2Motor;

    private final DCMotorSim j1Sim;
    private final DCMotorSim j2Sim;
    
    private final LinearSystem<N2, N1, N2> j1Plant; 
    private final LinearSystem<N2, N1, N2> j2Plant; 

    private final SingleJointedArmSim shoulderSim; 
    private final SingleJointedArmSim elbowSim; 

    private final TalonFXSimState j1ControllerSimState;
    private final TalonFXSimState j2ControllerSimState;

    
    public DJArmSimulations(DCMotor j1GearBox, DCMotor j2GearBox, TalonFX j1Controller, TalonFX j2Controller) {
        j1Motor = j1GearBox;
        j2Motor = j2GearBox;
        
        j1Plant = LinearSystemId.createSingleJointedArmSystem(
            j1Motor, 
            SingleJointedArmSim.estimateMOI(ArmConstants.SHOULDER_LENGTH, ArmConstants.SHOULDER_MASS), 
            ArmConstants.GEARING_SH);
        shoulderSim = new SingleJointedArmSim(
            j1Plant, j1Motor, 
            1,  // gearing is defined in j1Plant, i think i shouldnt re-enter gearing
            ArmConstants.SHOULDER_LENGTH, 
            ArmConstants.SH_MIN_ANGLE_RADS, ArmConstants.SH_MAX_ANGLE_RADS, true, 0,
            0.0 // Add noise with a std-dev of 1 tick
            );
        j1Sim = new DCMotorSim(j1Plant, j2Motor);

        j2Plant = LinearSystemId.createSingleJointedArmSystem(
            j2Motor, 
            SingleJointedArmSim.estimateMOI(ArmConstants.ELBOW_LENGTH, ArmConstants.ELBOW_MASS), 
            ArmConstants.GEARING_EL);
        elbowSim = new SingleJointedArmSim(
            j2Plant, j2Motor, 
            1, 
            ArmConstants.ELBOW_LENGTH, 
            ArmConstants.EL_MIN_ANGLE_RADS, ArmConstants.EL_MAX_ANGLE_RADS, true, 0, 
            0.0);
        j2Sim = new DCMotorSim(j2Plant, j2Motor);

        j1ControllerSimState = j1Controller.getSimState();
        j2ControllerSimState = j2Controller.getSimState();        
        
    }

    public SingleJointedArmSim shoulder() {return shoulderSim;}

    public SingleJointedArmSim elbow() {return elbowSim;}

    /** Reach goal. Goals should be angles in degrees, and will be converted to raw rotation units in the function */
    public void reachGoal(double j1Goal, double j2Goal) {
        // BÖYLE Mİ YAPICAM BİLMİYORUM
        j1ControllerSimState.addRotorPosition(j1Goal / ArmConstants.SH_ENC_CONV_FACT);
        j2ControllerSimState.addRotorPosition(j2Goal / ArmConstants.EL_ENC_CONV_FACT);
    }

    /**
     * Update simulations of what the arm is doing.
     */
    public void update() {
        // Set input voltages
        shoulderSim.setInputVoltage(j1ControllerSimState.getMotorVoltage());
        elbowSim.setInputVoltage(j2ControllerSimState.getMotorVoltage());

        // Update
        shoulderSim.update(0.02);
        elbowSim.update(0.02);

        // Napıyorum
        // We set our simulated encoder's readings and simulated battery voltage
        j1ControllerSimState.setRawRotorPosition(j1Sim.getAngularPositionRotations() / ArmConstants.GEARING_SH);
        j2ControllerSimState.setRawRotorPosition(j2Sim.getAngularPositionRotations() / ArmConstants.GEARING_EL);
    
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(shoulderSim.getCurrentDrawAmps() + elbowSim.getCurrentDrawAmps()));

        // Update telemetry
        SmartDashboard.putNumber("Arm Angle Joint 1", Units.radiansToDegrees(shoulderSim.getAngleRads()));
        SmartDashboard.putNumber("Arm Angle Joint 2", Units.radiansToDegrees(elbowSim.getAngleRads()));
    }

}
