package frc.robot.subsystems.arm;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;


/**
 * Mantıksız olabilir baba
 * 
 */
public class HalfCircleEncoder implements AutoCloseable, Sendable {
    private final DutyCycle m_dutyCycle;
    private boolean m_ownsDutyCycle;
    /**
     * Main difference from duty cycle encoder. A full cycle of the encoder is a half circle,
     * therefore, the first cycle is phase A (true) and second cycle is phase B (false)
     * 
     * So, if full range is set 180, the first 180deg is 0-180/true the second 180 (180-360) is 0-180/false
     */
    public boolean phase = true;
    private DigitalInput m_digitalInput;
    private int m_frequencyThreshold = 100;
    private double m_fullRange;
    private double m_expectedZero;
    private double m_periodNanos;
    private double m_sensorMin;
    private double m_sensorMax = 1.0;
    private double m_memory;
    private boolean m_isInverted;

    private SimDevice m_simDevice;
    private SimDouble m_simPosition;
    private SimBoolean m_simIsConnected;

    public HalfCircleEncoder(int channel, double fullRange, double expectedZero) {
        m_digitalInput = new DigitalInput(channel);
        m_ownsDutyCycle = true;
        m_dutyCycle = new DutyCycle(m_digitalInput);
        init(fullRange, expectedZero);
    }

    private void init(double fullRange, double expectedZero) {
    m_simDevice = SimDevice.create("DutyCycle:DutyCycleEncoder", m_dutyCycle.getSourceChannel());

    if (m_simDevice != null) {
      m_simPosition = m_simDevice.createDouble("Position", SimDevice.Direction.kInput, 0.0);
      m_simIsConnected = m_simDevice.createBoolean("Connected", SimDevice.Direction.kInput, true);
    }

    m_fullRange = fullRange * 2;
    m_expectedZero = expectedZero;

    SendableRegistry.addLW(this, "DutyCycle Encoder", m_dutyCycle.getSourceChannel());
    }

    private double mapSensorRange(double pos) {
        // map sensor range
        if (pos < m_sensorMin) {
          pos = m_sensorMin;
        }
        if (pos > m_sensorMax) {
          pos = m_sensorMax;
        }
        pos = (pos - m_sensorMin) / (m_sensorMax - m_sensorMin);
        return pos;
    }

    /**
     * Get the encoder value since the last reset.
     *
     * <p>This is reported in rotations since the last reset.
     *
     * @return the encoder value in rotations
     */
    public double get() {
      if (m_simPosition != null) {
        return m_simPosition.get();
      }

      double pos;
      // Compute output percentage (0-1)
      if (m_periodNanos == 0.0) {
        pos = m_dutyCycle.getOutput();
      } else {
        int highTime = m_dutyCycle.getHighTimeNanoseconds();
        pos = highTime / m_periodNanos;
      }

      // Map sensor range if range isn't full
      pos = mapSensorRange(pos);

      // Compute full range and offset
      pos = pos * (m_fullRange / 2) - m_expectedZero;

      // Map from 0 - Full Range
      double result = MathUtil.inputModulus(pos, 0, (m_fullRange / 2));
      phase = phaseShift(result);
      if (!phase) {
        if (m_isInverted) {
            m_memory = m_fullRange - result;
            return m_fullRange - result;
          }
          m_memory = result;
          return result;
      }
      // Invert if necessary
      if (m_isInverted) {
        m_memory = (m_fullRange / 2) - result;
        return (m_fullRange / 2) - result;
      }
      m_memory = result;
      return result;
    }

    private boolean phaseShift(double pos) {
        if (m_memory < (m_fullRange / 2) && pos > (m_fullRange / 2)) {
            return !phase;
        }
        return phase;
    }

    /**
    * Set the encoder duty cycle range. As the encoder needs to maintain a duty cycle, the duty cycle
    * cannot go all the way to 0% or all the way to 100%. For example, an encoder with a 4096 us
    * period might have a minimum duty cycle of 1 us / 4096 us and a maximum duty cycle of 4095 /
    * 4096 us. Setting the range will result in an encoder duty cycle less than or equal to the
    * minimum being output as 0 rotation, the duty cycle greater than or equal to the maximum being
    * output as 1 rotation, and values in between linearly scaled from 0 to 1.
    *
    * @param min minimum duty cycle (0-1 range)
    * @param max maximum duty cycle (0-1 range)
    */
    public void setDutyCycleRange(double min, double max) {
      m_sensorMin = MathUtil.clamp(min, 0.0, 1.0);
      m_sensorMax = MathUtil.clamp(max, 0.0, 1.0);
    }

    /**
     * Get the frequency in Hz of the duty cycle signal from the encoder.
     *
     * @return duty cycle frequency in Hz
     */
    public int getFrequency() {
      return m_dutyCycle.getFrequency();
    }

    /**
     * Get if the sensor is connected
     *
     * <p>This uses the duty cycle frequency to determine if the sensor is connected. By default, a
     * value of 100 Hz is used as the threshold, and this value can be changed with {@link
     * #setConnectedFrequencyThreshold(int)}.
     *
     * @return true if the sensor is connected
     */
    public boolean isConnected() {
      if (m_simIsConnected != null) {
        return m_simIsConnected.get();
      }
      return getFrequency() > m_frequencyThreshold;
    }

    /**
     * Change the frequency threshold for detecting connection used by {@link #isConnected()}.
     *
     * @param frequency the minimum frequency in Hz.
     */
    public void setConnectedFrequencyThreshold(int frequency) {
      if (frequency < 0) {
        frequency = 0;
      }

      m_frequencyThreshold = frequency;
    }

    /**
     * Sets the assumed frequency of the connected device.
     *
     * <p>By default, the DutyCycle engine has to compute the frequency of the input signal. This can
     * result in both delayed readings and jumpy readings. To solve this, you can pass the expected
     * frequency of the sensor to this function. This will use that frequency to compute the DutyCycle
     * percentage, rather than the computed frequency.
     *
     * @param frequency the assumed frequency of the sensor
     */
    public void setAssumedFrequency(double frequency) {
      if (frequency == 0.0) {
        m_periodNanos = 0.0;
      } else {
        m_periodNanos = 1000000000 / frequency;
      }
    }

    /**
     * Set if this encoder is inverted.
     *
     * @param inverted true to invert the encoder, false otherwise
     */
    public void setInverted(boolean inverted) {
      m_isInverted = inverted;
    }

    @Override
    public void close() {
      if (m_ownsDutyCycle) {
        m_dutyCycle.close();
      }
      if (m_digitalInput != null) {
        m_digitalInput.close();
      }
      if (m_simDevice != null) {
        m_simDevice.close();
      }
    }

    /**
     * Get the FPGA index for the DutyCycleEncoder.
     *
     * @return the FPGA index
     */
    public int getFPGAIndex() {
      return m_dutyCycle.getFPGAIndex();
    }

    /**
     * Get the channel of the source.
     *
     * @return the source channel
     */
    public int getSourceChannel() {
      return m_dutyCycle.getSourceChannel();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AbsoluteEncoder");
        builder.addDoubleProperty("Position", this::get, null);
        builder.addBooleanProperty("Is Connected", this::isConnected, null);
    }
}
