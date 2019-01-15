package org.usfirst.frc4388.utility;

public interface ControlLoopable 
{
	public void controlLoopUpdate();
	public void setPeriodMs(long periodMs);
}