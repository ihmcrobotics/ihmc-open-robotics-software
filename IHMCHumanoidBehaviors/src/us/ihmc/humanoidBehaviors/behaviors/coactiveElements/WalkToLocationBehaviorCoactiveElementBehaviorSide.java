package us.ihmc.humanoidBehaviors.behaviors.coactiveElements;

import javax.vecmath.Point2d;

import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.WalkToLocationBehavior;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class WalkToLocationBehaviorCoactiveElementBehaviorSide extends WalkToLocationBehaviorCoactiveElement
{
   private WalkToLocationBehavior walkToLocationBehavior;

   public void setWalkToBehavior(WalkToLocationBehavior walkToLocationBehavior)
   {
      this.walkToLocationBehavior = walkToLocationBehavior;
   }

   @Override
   public void initializeUserInterfaceSide()
   {
   }

   @Override
   public void updateUserInterfaceSide()
   {
   }

   @Override
   public void initializeMachineSide()
   {
   }

   @Override
   public void updateMachineSide()
   {
      if (locationSet.getBooleanValue() && !walking.getBooleanValue())
      {
         //setup the walk to location and tell the behavior it is ready.
         walkToLocationBehavior.setTarget(new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(x.getDoubleValue(), y.getDoubleValue()), 0));
         walking.set(true);
      }
   }
}
