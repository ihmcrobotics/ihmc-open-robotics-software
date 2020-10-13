package us.ihmc.simulationConstructionSetTools.util.gui;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoVariableTogglerOutlet
{
   private final YoEnum<ToggleMode> toggleMode;

   public YoVariableTogglerOutlet(String name, YoRegistry parent)
   {
      toggleMode = new YoEnum<ToggleMode>(name, parent, ToggleMode.class);
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
