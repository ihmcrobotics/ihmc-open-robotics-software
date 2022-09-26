package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class SettableFootSwitch implements FootSwitchInterface
{
   YoBoolean hasFootHitGround;
   private final ContactablePlaneBody foot;
   private final double totalRobotWeight;
   private final int totalNumberOfFeet;

   public SettableFootSwitch(ContactablePlaneBody foot, double totalRobotWeight, int totalNumberOfFeet, YoRegistry registry)
   {
      this.totalNumberOfFeet = totalNumberOfFeet;
      this.hasFootHitGround = new YoBoolean(foot.getName() + "_SettableFootSwitch", registry);
      this.totalRobotWeight = totalRobotWeight;
      this.foot = foot;
      hasFootHitGround.set(false);
   }

   @Override
   public boolean hasFootHitGround()
   {
      return hasFootHitGround.getBooleanValue();
   }

   @Override
   public double getFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public FramePoint2DReadOnly getCenterOfPressure()
   {
      return null;
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setToZero();
      if (hasFootHitGround())
         footWrenchToPack.setLinearPartZ(totalRobotWeight / totalNumberOfFeet);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return foot.getSoleFrame();
   }

   @Override
   public void reset()
   {
      hasFootHitGround.set(false);
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return false;
   }

   public void setFootContactState(boolean hasFootHitGround)
   {
      this.hasFootHitGround.set(hasFootHitGround);
   }
}
