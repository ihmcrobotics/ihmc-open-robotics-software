package us.ihmc.simulationConstructionSetTools.joystick;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public class EnumYoVariableJoystickEventListener<T extends Enum<T>> implements JoystickEventListener
{
   private final YoEnum<T> yoEnum;
   private final Component component;
   private final T enumToSwitchTo;
   
   public EnumYoVariableJoystickEventListener(YoEnum<T> yoEnum, Component component, T enumToSwitchTo)
   {
      this.yoEnum = yoEnum;
      this.component = component;
      this.enumToSwitchTo = enumToSwitchTo;
   }
   
   @Override
   public void processEvent(Event event)
   {
      if (event.getComponent() == component)
      {
         yoEnum.set(enumToSwitchTo);
      }
   }
}
