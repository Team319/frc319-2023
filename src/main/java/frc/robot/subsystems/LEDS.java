// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {

  private static final int length = 1;//117;

  public final AddressableLED m_led = new AddressableLED(0);
  public final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(length);

  private Alliance alliance = Alliance.Invalid;

  
  /** Creates a new LEDS. */
  public LEDS() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void periodic(){

    if(DriverStation.isFMSAttached()){
      alliance = DriverStation.getAlliance();
    }

    //Select LED Mode
    //solid(Section.FULL, Color.kRed);

  }

  public void colorTest(int rValue, int gValue, int bValue) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, rValue, gValue, bValue);
    }

    m_led.setData(m_ledBuffer);
  }

  public void solid(Section section, Color color){
    for (int i = section.start(); i < section.end(); i++){
      m_ledBuffer.setLED(i, color);
    }
  }

  public void solidRGB(Section section, int r, int g, int b){
    for (int i = section.start(); i < section.end(); i++){
      m_ledBuffer.setRGB(i, r, g, b );
    }
  }

  public static enum Section {
    STATIC,
    SHOULDER,
    FULL,
    STATIC_LOW,
    STATIC_MID,
    STATIC_HIGH;
  

  private int start(){
    switch(this) {
      case STATIC:
        return 0;
      default:
        return 0;
    }
  }

  private int end(){
    switch(this) {
      default:
        return length;
    }
  }

  }

}
