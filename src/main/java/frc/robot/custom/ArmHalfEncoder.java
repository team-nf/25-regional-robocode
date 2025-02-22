// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.custom;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class ArmHalfEncoder {

    private DutyCycleEncoder m_encoder; 
    private boolean firstHalf = true;
    private double startAngle = 0.0;
    private double lastRawAngle = 0.0;
    private double jointAngle = 0.0;

    public ArmHalfEncoder(int id) {
        m_encoder = new DutyCycleEncoder(id);
        startAngle = m_encoder.get()/2;
    }

    public double getAngle() {
        return jointAngle;
    }

    public double getRawEncoder() {
        return m_encoder.get();
    }

    public void periodic()
    {
        if(Math.abs(lastRawAngle - m_encoder.get()) > 350) firstHalf = !firstHalf;
        lastRawAngle = m_encoder.get();
        jointAngle = (m_encoder.get()/2 + (firstHalf ? 0 : 180)) - startAngle;
        if (jointAngle < 0) jointAngle += 360;
    }

    public void reset() {
        firstHalf = true;
        startAngle = m_encoder.get()/2;
        lastRawAngle = m_encoder.get();
    }

    public DutyCycleEncoder getEncoder() {
        return m_encoder;
    }

    public double getStartAngle() {
        return startAngle;
    }
}
