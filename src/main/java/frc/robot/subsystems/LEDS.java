// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.VirtualSubsystem;

public class LEDS extends VirtualSubsystem {
  private static LEDS instance; 

  public static LEDS getInstance(){
    if(instance == null){
      instance = new LEDS();
    }
    return instance;
  }
  private static final int length = 117;

  public final AddressableLED m_led;
  public final AddressableLEDBuffer m_ledBuffer;

  private Alliance alliance = Alliance.Invalid;

  
  /** Creates a new LEDS. */
  public LEDS() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(length);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void periodic(){

    // if(DriverStation.isFMSAttached()){
    //   alliance = DriverStation.getAlliance();
    // }

    //Select LED Mode
    solid(Color.kRed);
    //strobe(Section.FULL,Color.kBlue, 0.1);

  }

  public void colorTest(int rValue, int gValue, int bValue) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, rValue, gValue, bValue);
    }

    m_led.setData(m_ledBuffer);
  }

  public void solid(Color color){
    for (int i = 1; i < length; i++){
      m_ledBuffer.setLED(i, color);
    }
  }

  public void strobe(Section section, Color color, double durration){
    boolean on = ((Timer.getFPGATimestamp() % durration)/durration) > .5;
    solid(on ? color : color.kBlack);
  }

  public void solidRGB(Section section, int r, int g, int b){
    for (int i = section.start(); i < section.end(); i++){
      m_ledBuffer.setRGB(i, r, g, b );
    }
  }

  // Taken from 6328 - L.S
  // Possilbe strobe?
  public void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), 0.4) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), 0.4) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        m_ledBuffer.setLED(i, new Color(red, green, blue));
      }
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
