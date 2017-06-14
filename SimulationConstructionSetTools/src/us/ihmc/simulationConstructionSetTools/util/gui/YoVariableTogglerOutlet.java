package us.ihmc.simulationConstructionSetTools.util.gui;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.EnumYoVariable;


public class YoVariableTogglerOutlet
{
   private final EnumYoVariable<ToggleMode> toggleMode;

   public YoVariableTogglerOutlet(String name, YoVariableRegistry parent)
   {
      toggleMode = new EnumYoVariable<ToggleMode>(name, parent, ToggleMode.class);
      toggleMode.set(ToggleMode.NO_CHANGE);
   }

   public void reset()
   {
      toggleMode.set(ToggleMode.NO_CHANGE);
   }

   public ToggleMode getToggleMode()
   {
      return toggleMode.getEnumValue();
   }
}
