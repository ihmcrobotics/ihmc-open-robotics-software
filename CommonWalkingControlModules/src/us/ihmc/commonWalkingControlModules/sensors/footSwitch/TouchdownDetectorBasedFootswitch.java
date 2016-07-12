package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

import java.util.ArrayList;

public abstract class TouchdownDetectorBasedFootswitch implements FootSwitchInterface
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + "Registry");
   protected final ArrayList<TouchdownDetector> touchdownDetectors = new ArrayList<>();

   protected final BooleanYoVariable hasTouchedDown;

   public TouchdownDetectorBasedFootswitch(String name, YoVariableRegistry parentRegistry)
   {
      hasTouchedDown = new BooleanYoVariable(name + "_hasTouchedDown", registry);
      setupTouchdownDetectors(touchdownDetectors);

      parentRegistry.addChild(registry);
   }

   abstract protected void setupTouchdownDetectors(ArrayList<TouchdownDetector> touchdownDetectors);

   @Override
   public boolean hasFootHitGround()
   {
      boolean ret = true;

      for(TouchdownDetector touchdownDetector : touchdownDetectors)
         ret &= touchdownDetector.hasTouchedDown();

      hasTouchedDown.set(ret);

      return hasTouchedDown.getBooleanValue();
   }

   @Override
   public void reset()
   {
      hasTouchedDown.set(false);
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return false;
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      hasTouchedDown.set(hasFootHitGround);
   }
}
