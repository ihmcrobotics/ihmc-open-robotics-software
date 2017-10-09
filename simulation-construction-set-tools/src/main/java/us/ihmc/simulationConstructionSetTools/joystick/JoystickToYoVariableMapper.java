package us.ihmc.simulationConstructionSetTools.joystick;

import java.util.ArrayList;
import java.util.HashMap;

import net.java.games.input.Component;
import us.ihmc.commons.PrintTools;
import us.ihmc.yoVariables.dataBuffer.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public class JoystickToYoVariableMapper
{
   private final YoVariableHolder yoVariableHolder;
   private final ArrayList<JoystickEventListener> eventListeners;
   private final HashMap<Component, JoystickEventListener> componentToEventListenerMap = new HashMap<>();
   
   public JoystickToYoVariableMapper(YoVariableHolder yoVariableHolder, ArrayList<JoystickEventListener> eventListeners)
   {
      this.yoVariableHolder = yoVariableHolder;
      this.eventListeners = eventListeners;
   }
   
   public void mapDoubleYoVariableToComponent(Component component, String variableName, double minValue, double maxValue, double deadZone, boolean invert)
   {
      if (component != null)
      {
         YoDouble yoVariable = (YoDouble) yoVariableHolder.getVariable(variableName);
         if (yoVariable != null)
         {
            DoubleYoVariableJoystickEventListener joystickEventListener = new DoubleYoVariableJoystickEventListener(yoVariable, component, minValue, maxValue, deadZone, invert);
            eventListeners.add(joystickEventListener);
            componentToEventListenerMap.put(component, joystickEventListener);
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
         YoBoolean yoVariable = (YoBoolean) yoVariableHolder.getVariable(variableName);
         if (yoVariable != null)
         {
            BooleanYoVariableJoystickEventListener joystickEventListener = new BooleanYoVariableJoystickEventListener(yoVariable, component, toggle, flip);
            eventListeners.add(joystickEventListener);
            componentToEventListenerMap.put(component, joystickEventListener);
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
         YoEnum<T> yoVariable = (YoEnum<T>) yoVariableHolder.getVariable(variableName);
         if (yoVariable != null)
         {
            EnumYoVariableJoystickEventListener<T> joystickEventListener = new EnumYoVariableJoystickEventListener<T>(yoVariable, component, enumToSwitchTo);
            eventListeners.add(joystickEventListener);
            componentToEventListenerMap.put(component, joystickEventListener);
         }
         else
         {
            PrintTools.warn(this, "Variable " + variableName + " could not be found!");
         }
      }
   }
   
   public void removeComponentJoystickEventListener(Component component)
   {
      eventListeners.remove(componentToEventListenerMap.get(component));
   }
   
   public void addComponentJoystickEventListener(Component component)
   {
      eventListeners.add(componentToEventListenerMap.get(component));
   }
}
