package us.ihmc.simulationconstructionset.joystick;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

public class EnumYoVariableDependentJoystickInputManager<T>
{
   private static final int JOYSTICK_POLL_INTERVAL_MS = 20;
   private final List<Joystick> joysticks = new ArrayList<>();
   private final HashMap<Enum<?>, ArrayList<JoystickEventListener>> eventListeners = new HashMap<>();
   private final EnumYoVariable<?> enumYoVariable;
   private final T[] enumValues;
   
   public EnumYoVariableDependentJoystickInputManager(final EnumYoVariable<?> enumYoVariable, Class<T> enumType, boolean createFirstJoystick) throws JoystickNotFoundException
   {
      this.enumValues = enumType.getEnumConstants();
      this.enumYoVariable = enumYoVariable;
      
      if (createFirstJoystick)
      {
         joysticks.add(new Joystick());
         joysticks.get(joysticks.size() - 1).setPollInterval(JOYSTICK_POLL_INTERVAL_MS);
      }
      
      enumYoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            updateListeners(enumYoVariable);
         }
      });
   }
   
   public void addJoystick(JoystickModel joystickModel, int indexFoundOnSystem) throws JoystickNotFoundException
   {
      joysticks.add(new Joystick(joystickModel, indexFoundOnSystem));
      joysticks.get(joysticks.size() - 1).setPollInterval(JOYSTICK_POLL_INTERVAL_MS);
   }
   
   public void addFirstJoystickFoundOnSystem() throws JoystickNotFoundException
   {
      joysticks.add(new Joystick());
      joysticks.get(joysticks.size() - 1).setPollInterval(JOYSTICK_POLL_INTERVAL_MS);
   }
   
   public Joystick getFirstJoystick()
   {
      return joysticks.get(0);
   }
   
   public Joystick getJoystick(int index)
   {
      return joysticks.get(index);
   }
   
   public int getNumberOfJoysticks()
   {
      return joysticks.size();
   }
   
   public void initialize()
   {
      updateListeners(enumYoVariable);
   }
   
   private void updateListeners(final EnumYoVariable<?> enumYoVariable)
   {
      for (Joystick joystick : joysticks)
      {
         joystick.clearEventListeners();
         T enumValue = enumValues[enumYoVariable.getOrdinal()];
         if(eventListeners.containsKey(enumValue))
         {
            for(JoystickEventListener eventListener : eventListeners.get(enumValue))
            {
               joystick.addJoystickEventListener(eventListener);
            }
         }
      }
   }
   
   public void disableJoysticks()
   {
      for (Joystick joystick : joysticks)
      {
         joystick.clearEventListeners();
      }
   }
   
   public void addJoystickMapping(EnumDependentJoystickMapping joystickMap)
   {
      eventListeners.put(joystickMap.getEnum(), joystickMap.getEventListeners());
   }
}
