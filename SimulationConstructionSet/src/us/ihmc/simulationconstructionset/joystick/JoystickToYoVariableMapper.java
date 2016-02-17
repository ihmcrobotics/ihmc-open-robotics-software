package us.ihmc.simulationconstructionset.joystick;

import java.util.ArrayList;

import net.java.games.input.Component;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.io.printing.PrintTools;

public class JoystickToYoVariableMapper
{
   private final YoVariableHolder yoVariableHolder;
   private final ArrayList<JoystickEventListener> eventListeners;
   
   public JoystickToYoVariableMapper(YoVariableHolder yoVariableHolder, ArrayList<JoystickEventListener> eventListeners)
   {
      this.yoVariableHolder = yoVariableHolder;
      this.eventListeners = eventListeners;
   }
   
   public void mapDoubleYoVariableToComponent(Component component, String variableName, double minValue, double maxValue, double deadZone, boolean invert)
   {
      if (component != null)
      {
         DoubleYoVariable yoVariable = (DoubleYoVariable) yoVariableHolder.getVariable(variableName);
         if (yoVariable != null)
         {
            eventListeners.add(new DoubleYoVariableJoystickEventListener(yoVariable, component, minValue, maxValue, deadZone, invert));
         }
         else
         {
            PrintTools.warn(this, "Variable " + variableName + " could not be found!");
         }
      }
   }
   
   public void mapBooleanYoVariableToComponent(Component component, String variableName, boolean toggle, boolean flip)
   {
      if (component != null)
      {
         BooleanYoVariable yoVariable = (BooleanYoVariable) yoVariableHolder.getVariable(variableName);
         if (yoVariable != null)
         {
            eventListeners.add(new BooleanYoVariableJoystickEventListener(yoVariable, component, toggle, flip));
         }
         else
         {
            PrintTools.warn(this, "Variable " + variableName + " could not be found!");
         }
      }
   }
   
   @SuppressWarnings("unchecked")
   public <T extends Enum<T>> void mapEnumYoVariableToComponent(Component component, String variableName, T enumToSwitchTo)
   {
      if (component != null)
      {
         EnumYoVariable<T> yoVariable = (EnumYoVariable<T>) yoVariableHolder.getVariable(variableName);
         if (yoVariable != null)
         {
            eventListeners.add(new EnumYoVariableJoystickEventListener<T>(yoVariable, component, enumToSwitchTo));
         }
         else
         {
            PrintTools.warn(this, "Variable " + variableName + " could not be found!");
         }
      }
   }
}
