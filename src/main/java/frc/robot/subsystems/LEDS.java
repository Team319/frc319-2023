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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ledTest() {
    m_led.setLength(m_ledBuffer.getLength());
  }
}
