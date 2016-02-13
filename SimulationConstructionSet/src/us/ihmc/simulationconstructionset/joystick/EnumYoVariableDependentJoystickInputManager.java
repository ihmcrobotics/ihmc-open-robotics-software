package us.ihmc.simulationconstructionset.joystick;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;

public class EnumYoVariableDependentJoystickInputManager<T>
{
   private final int pollIntervalMillis = 20;
   private final Joystick rightJoystick;
   private final Joystick leftJoystick;
   private final HashMap<Enum<?>, ArrayList<JoystickEventListener>> eventListeners = new HashMap<>();
   private final EnumYoVariable<?> enumYoVariable;
   private final T[] enumValues;
   
   public EnumYoVariableDependentJoystickInputManager(final EnumYoVariable<?> enumYoVariable, Class<T> enumType) throws IOException
   {
      this.enumValues = enumType.getEnumConstants();
      this.enumYoVariable = enumYoVariable;
      
      rightJoystick = new Joystick(JoystickModel.MAD_CATZ_FLY5_STICK, 0);
      rightJoystick.setPollInterval(pollIntervalMillis);
      leftJoystick = new Joystick(JoystickModel.MAD_CATZ_V1_STICK, 0);
      leftJoystick.setPollInterval(pollIntervalMillis);
      
      enumYoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            updateListeners(enumYoVariable);
         }
      });
   }
   
   public Joystick getRightJoystick()
   {
      return rightJoystick;
   }
   
   public Joystick getLeftJoystick()
   {
      return leftJoystick;
   }
   
   public void initialize()
   {
      updateListeners(enumYoVariable);
   }
   
   private void updateListeners(final EnumYoVariable<?> enumYoVariable)
   {
      rightJoystick.clearEventListeners();
      T enumValue = enumValues[enumYoVariable.getOrdinal()];
      if(eventListeners.containsKey(enumValue))
      {
         for(JoystickEventListener eventListener : eventListeners.get(enumValue))
         {
            rightJoystick.addJoystickEventListener(eventListener);
         }
      }
   }
   
   public void disableJoystick()
   {
      rightJoystick.clearEventListeners();
   }
   
   public void addJoystickMapping(EnumDependentJoystickMapping joystickMap)
   {
      eventListeners.put(joystickMap.getEnum(), joystickMap.getEventListeners());
   }
}
