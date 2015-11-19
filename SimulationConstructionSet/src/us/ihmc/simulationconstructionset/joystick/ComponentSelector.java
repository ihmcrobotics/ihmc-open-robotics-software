package us.ihmc.simulationconstructionset.joystick;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;

public interface ComponentSelector
{
   public void listComponents();
   public Component findComponent(Identifier identifier) throws JoystickComponentNotFoundException;
}