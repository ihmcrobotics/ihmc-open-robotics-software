package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class SettableFootSwitch implements FootSwitchInterface
{
   YoBoolean hasFootHitGround;
   private final ContactablePlaneBody foot;
   private final double totalRobotWeight;
   private final YoFramePoint2D yoResolvedCoP;
   private final int totalNumberOfFeet;

   public SettableFootSwitch(ContactablePlaneBody foot, double totalRobotWeight, int totalNumberOfFeet, YoVariableRegistry registry)
   {
      this.totalNumberOfFeet = totalNumberOfFeet;
      this.hasFootHitGround = new YoBoolean(foot.getName() + "_SettableFootSwitch", registry);
      this.totalRobotWeight = totalRobotWeight;
      this.foot = foot;
      hasFootHitGround.set(false);
      yoResolvedCoP = new YoFramePoint2D(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);
   }
   
   @Override
   public boolean hasFootHitGround()
   {
      return hasFootHitGround.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
   {
      copToPack.setToNaN(getMeasurementFrame());
   }

   @Override
   public void updateCoP()
   {
      yoResolvedCoP.setToZero();
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
   
   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      this.hasFootHitGround.set(hasFootHitGround);
   }

   @Override
   public void trustFootSwitchInSwing(boolean trustFootSwitch)
   {

   }

   @Override
   public void trustFootSwitchInSupport(boolean trustFootSwitch)
   {

   }
}
