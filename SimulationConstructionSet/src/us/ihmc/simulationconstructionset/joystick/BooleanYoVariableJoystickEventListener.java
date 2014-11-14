package us.ihmc.simulationconstructionset.joystick;

import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import net.java.games.input.Component;
import net.java.games.input.Event;


public class BooleanYoVariableJoystickEventListener implements JoystickEventListener
{
   private final BooleanYoVariable variable;
   private final Component component;
   private final boolean flip;

   public BooleanYoVariableJoystickEventListener(BooleanYoVariable variable, Component component, boolean flip)
   {
      if (component.isAnalog())
         throw new RuntimeException("component is analog; should be digital (i.e. an on/off button)");
      this.variable = variable;
      this.component = component;
      this.flip = flip;
   }

   public void processEvent(Event event)
   {
      if (event.getComponent() == component)
      {
         boolean value = event.getValue() == 1.0f;
         variable.set(value ^ flip);
      }
   }
}
