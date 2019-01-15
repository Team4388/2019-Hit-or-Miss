package org.usfirst.frc4388.utility;

import java.util.Timer;
import java.util.TimerTask;
import java.util.Vector;

/**
 * ControlLooper.java
 * <p>
 * Runs several ControlLoopables simultaneously with one timing loop.
 * Useful for running a bunch of control loops
 * with only one Thread worth of overhead.
 * 
 * Shamelessly stolen from team 254 2015 code and then slightly modified
 *
 * @author Tom Bottiglieri
 */

public class ControlLooper 
{
	private Timer controlLoopTimer;	
    private Vector<ControlLoopable> loopables = new Vector<ControlLoopable>();
	private long periodMs;
	private String name;

	public ControlLooper(String name, long periodMs) {
		this.name = name;
		this.periodMs = periodMs;
	}
	
	private class ControlLoopTask extends TimerTask {
		private ControlLooper controlLooper;

		public ControlLoopTask(ControlLooper controlLooper) {
			if (controlLooper == null) {
				throw new NullPointerException("Given control looper was null");
			}
			this.controlLooper = controlLooper;
		}

		@Override
		public void run() {
			controlLooper.controlLoopUpdate();
		}
		
	}
	
	public String getName() {
		return name;
	}
	
	public void start() {
		if (controlLoopTimer == null) {
			controlLoopTimer = new Timer();
			controlLoopTimer.schedule(new ControlLoopTask(this), 0L, periodMs);
		}
	}
	
	public void stop() {
		if (controlLoopTimer != null) {
			controlLoopTimer.cancel();
			controlLoopTimer = null;
		}
	}
		
	private void controlLoopUpdate() {
        for (int i = 0; i < loopables.size(); ++i) {
        	ControlLoopable c = loopables.elementAt(i);
            if (c != null) {
            	c.controlLoopUpdate();
            }
        }
	}
	
    public void addLoopable(ControlLoopable c) {
        loopables.addElement(c);
        c.setPeriodMs(periodMs);
    }
}