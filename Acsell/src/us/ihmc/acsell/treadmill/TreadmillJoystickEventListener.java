package us.ihmc.acsell.treadmill;

import java.io.IOException;
import java.io.OutputStream;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public class TreadmillJoystickEventListener implements JoystickEventListener
{
	final boolean debug = false;
	final OutputStream out;
	private int currentSpeed = 0;
	public TreadmillJoystickEventListener(OutputStream out)
	{
		this.out = out;
	}
	
	@Override
	public void processEvent(Event event) {
		if(debug) System.out.println(event.toString());		
		try {	
			if (event.getComponent().getIdentifier().equals(Component.Identifier.Button.BASE4) && event.getValue() == 1.0)
			{
				if(debug) System.out.println("Base 4 Pressed");
				out.write(TreadmillSerialManager.Commands.START_BELT);
				setSpeed(0);
			}
			if (event.getComponent().getIdentifier().equals(Component.Identifier.Button.BASE6) && event.getValue() == 1.0)
			{
				if(debug) System.out.println("Base 6 Pressed");
				out.write(TreadmillSerialManager.Commands.STOP_BELT);
				setSpeed(0);
			}
			if (event.getComponent().getIdentifier().equals(Component.Identifier.Button.BASE3) && event.getValue() == 1.0)
			{
				if(debug) System.out.println("Base 3 Pressed");
				increaseSpeed();
				out.write(TreadmillSerialManager.Commands.SET_SPEED);
				out.write(parseIntForSend(currentSpeed));
					
			}
			if (event.getComponent().getIdentifier().equals(Component.Identifier.Button.BASE5) && event.getValue() == 1.0)
			{
				if(debug) System.out.println("Base 5 Pressed");
				decreaseSpeed();
				out.write(TreadmillSerialManager.Commands.SET_SPEED);
				out.write(parseIntForSend(currentSpeed));	
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	private byte[] parseIntForSend(int v)
	{
		String s = String.format("%04d", v);
		char[] c = s.toCharArray();
		byte[] b = new byte[c.length];
		for (int i = 0; i<b.length;i++)
		{
			b[i] = (byte)c[i];
		}
		return b;
	}
	
	public void increaseSpeed()
	{
		if (currentSpeed<120)
			currentSpeed+=1;
	}
	
	public void decreaseSpeed()
	{
		if (currentSpeed>0)
			currentSpeed-=1;
	}
	
	public int getSpeed()
	{
		return currentSpeed;
	}
	
	public void setSpeed(int v)
	{
		if(v<121 & v>-1)
		{
			currentSpeed = v;
		}
	}
	
}