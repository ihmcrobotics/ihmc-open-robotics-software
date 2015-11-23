package us.ihmc.simulationconstructionset.joystick;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import us.ihmc.tools.inputDevices.joystick.JoystickComponentNotFoundException;

public interface ComponentSelector
{   
   public Component findComponent(Identifier identifier) throws JoystickComponentNotFoundException;
}