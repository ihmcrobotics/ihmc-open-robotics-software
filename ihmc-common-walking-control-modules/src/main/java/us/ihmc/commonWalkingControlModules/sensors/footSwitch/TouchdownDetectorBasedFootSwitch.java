package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.commonWalkingControlModules.touchdownDetector.NecessaryTouchdownDetectors;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.sensors.FootSwitchInterface;

import java.util.ArrayList;
import java.util.List;

public abstract class TouchdownDetectorBasedFootSwitch implements FootSwitchInterface
{
   protected final YoVariableRegistry registry;
   protected final List<TouchdownDetector> touchdownDetectors = new ArrayList<>();

   protected final YoBoolean controllerThinksHasTouchedDown;

   public TouchdownDetectorBasedFootSwitch(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = parentRegistry;
      controllerThinksHasTouchedDown = new YoBoolean(name + "_controllerThinksHasTouchedDown", parentRegistry);
   }

   @Override
   public void reset()
   {
      controllerThinksHasTouchedDown.set(false);
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return false;
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      controllerThinksHasTouchedDown.set(hasFootHitGround);
   }
}
