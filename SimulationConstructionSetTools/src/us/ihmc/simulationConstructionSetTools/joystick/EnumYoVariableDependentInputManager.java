package us.ihmc.simulationConstructionSetTools.joystick;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.EnumDependentSliderBoardMapping;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

public class EnumYoVariableDependentInputManager<T extends Enum<T>>
{
   private static final int JOYSTICK_POLL_INTERVAL_MS = 20;
   private final List<Joystick> joysticks = new ArrayList<>();
   private SliderBoardConfigurationManager sliderBoardConfigurationManager;
   private final HashMap<Enum<T>, ArrayList<JoystickEventListener>> joystickEventListeners = new HashMap<>();
   private final HashMap<Enum<T>, EnumDependentSliderBoardMapping<T>> sliderBoardConfigurations = new HashMap<>();
   private final EnumYoVariable<T> enumYoVariable;
   private final T[] enumValues;
   
   protected EnumYoVariableDependentInputManager(final EnumYoVariable<T> enumYoVariable, Class<T> enumType)
   {
      this.enumValues = enumType.getEnumConstants();
      this.enumYoVariable = enumYoVariable;
      
      enumYoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            updateListeners(enumYoVariable);
         }
      });
   }
   
   protected void addSliderBoardConfigurationManager(SliderBoardConfigurationManager sliderBoardConfigurationManager)
   {
      this.sliderBoardConfigurationManager = sliderBoardConfigurationManager;
   }
   
   protected void addSliderBoardMapping(EnumDependentSliderBoardMapping<T> enumDependentSliderBoardMapping)
   {
      if (sliderBoardConfigurationManager == null)
      {
         throw new RuntimeException("You must call addSliderBoardConfigurationManager(SliderBoardConfigurationManager) first.");
      }
      
      sliderBoardConfigurations.put(enumDependentSliderBoardMapping.getEnum(), enumDependentSliderBoardMapping);
   }
   
   protected void addJoystick(JoystickModel joystickModel, int indexFoundOnSystem) throws JoystickNotFoundException
   {
      addJoystick(new Joystick(joystickModel, indexFoundOnSystem));
   }
   
   protected void addFirstJoystickFoundOnSystem() throws JoystickNotFoundException
   {
      addJoystick(new Joystick());
   }
   
   private void addJoystick(Joystick joystick)
   {
      joysticks.add(joystick);
      joysticks.get(joysticks.size() - 1).setPollInterval(JOYSTICK_POLL_INTERVAL_MS);
   }
   
   public Joystick getFirstJoystick()
   {
      return joysticks.get(0);
   }
   
   protected void disableJoysticks()
   {
      for (Joystick joystick : joysticks)
      {
         joystick.clearEventListeners();
      }
   }

   protected void addJoystickMapping(EnumDependentJoystickMapping<T> joystickMap)
   {
      joystickEventListeners.put(joystickMap.getEnum(), joystickMap.getEventListeners());
   }

   public Joystick getJoystick(int index)
   {
      return joysticks.get(index);
   }
   
   public int getNumberOfJoysticks()
   {
      return joysticks.size();
   }
   
   protected void initialize()
   {
      updateListeners(enumYoVariable);
   }
   
   protected EnumYoVariable<T> getEnumYoVariable()
   {
      return enumYoVariable;
   }
   
   private void updateListeners(final EnumYoVariable<?> enumYoVariable)
   {
      T enumValue = enumValues[enumYoVariable.getOrdinal()];
      
      for (Joystick joystick : joysticks)
      {
         joystick.clearEventListeners();
         if(joystickEventListeners.containsKey(enumValue))
         {
            for(JoystickEventListener eventListener : joystickEventListeners.get(enumValue))
            {
               joystick.addJoystickEventListener(eventListener);
            }
         }
      }
      
      if (sliderBoardConfigurationManager != null && sliderBoardConfigurations.containsKey(enumValue))
      {
         sliderBoardConfigurations.get(enumValue).activateConfiguration(sliderBoardConfigurationManager);
      }
   }
}
