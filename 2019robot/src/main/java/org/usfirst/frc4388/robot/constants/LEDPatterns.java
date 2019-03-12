package org.usfirst.frc4388.robot.constants;

public enum LEDPatterns {
    RED(0.61f), BLACK(0.99f);
  
    private final float id;
    LEDPatterns(float id) { this.id = id; }
    public float getValue() { return id; }
  }