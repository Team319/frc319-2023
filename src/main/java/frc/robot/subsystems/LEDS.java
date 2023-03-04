// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {
  public AddressableLED m_led = new AddressableLED(9);
  public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

  
  /** Creates a new LEDS. */
  public LEDS() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void colorTest(int rValue, int gValue, int bValue) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, rValue, gValue, bValue);
    }

    m_led.setData(m_ledBuffer);
  }
}
