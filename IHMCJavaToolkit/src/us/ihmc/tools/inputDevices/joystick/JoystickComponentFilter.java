package us.ihmc.tools.inputDevices.joystick;

import net.java.games.input.Component.Identifier;

public interface JoystickComponentFilter
{
   public double apply(double value);
   
   public Identifier getIdentifier();
}
