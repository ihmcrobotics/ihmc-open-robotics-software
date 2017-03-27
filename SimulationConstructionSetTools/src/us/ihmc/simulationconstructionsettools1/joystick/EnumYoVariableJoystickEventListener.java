package us.ihmc.simulationconstructionsettools1.joystick;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public class EnumYoVariableJoystickEventListener<T extends Enum<T>> implements JoystickEventListener
{
   private final EnumYoVariable<T> enumYoVariable;
   private final Component component;
   private final T enumToSwitchTo;
   
   public EnumYoVariableJoystickEventListener(EnumYoVariable<T> enumYoVariable, Component component, T enumToSwitchTo)
   {
      this.enumYoVariable = enumYoVariable;
      this.component = component;
      this.enumToSwitchTo = enumToSwitchTo;
   }
   
   @Override
   public void processEvent(Event event)
   {
      if (event.getComponent() == component)
      {
         enumYoVariable.set(enumToSwitchTo);
      }
   }
}
