package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private static final int PH_CAN_ID = 3;
  private final PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);

  // First solenoid (ports 0,1)
  private final DoubleSolenoid m_solenoidA = m_pH.makeDoubleSolenoid(0, 1);

  // Second solenoid (ports 6,7)
  private final DoubleSolenoid m_solenoidB = m_pH.makeDoubleSolenoid(6, 7);

  public Pneumatics() {

  }

  public void setForward() {
      m_solenoidA.set(DoubleSolenoid.Value.kForward);
      m_solenoidB.set(DoubleSolenoid.Value.kForward);
  }

  public void setReverse() {
      m_solenoidA.set(DoubleSolenoid.Value.kReverse);
      m_solenoidB.set(DoubleSolenoid.Value.kReverse);
  }

  public void setOff() {
      m_solenoidA.set(DoubleSolenoid.Value.kOff);
      m_solenoidB.set(DoubleSolenoid.Value.kOff);
  }

  public void enableCompressor() {
    m_pH.enableCompressorAnalog(20, 60);  
  }

  public void disableCompressor() {
      m_pH.disableCompressor();
  }
}
