package us.ihmc.tools.inputDevices.joystick;

import net.java.games.input.Event;

public interface JoystickEventListener
{
   public abstract void processEvent(Event event);
}