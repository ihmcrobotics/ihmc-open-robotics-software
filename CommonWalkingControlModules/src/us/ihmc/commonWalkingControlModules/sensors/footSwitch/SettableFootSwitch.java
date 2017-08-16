package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class SettableFootSwitch implements FootSwitchInterface
{
   YoBoolean hasFootHitGround;
   private final ContactablePlaneBody foot;
   private final double totalRobotWeight;
   private final YoFramePoint2d yoResolvedCoP;

   public SettableFootSwitch(ContactablePlaneBody foot, RobotQuadrant quadrant, double totalRobotWeight, YoVariableRegistry registry)
   {
      this.hasFootHitGround = new YoBoolean(quadrant.getCamelCaseName() + "_SettableFootSwitch", registry);
      this.totalRobotWeight = totalRobotWeight;
      this.foot = foot;
      hasFootHitGround.set(false);
      yoResolvedCoP = new YoFramePoint2d(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);
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
         footWrenchToPack.setLinearPartZ(totalRobotWeight / 4.0);
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
   public void trustFootSwitch(boolean trustFootSwitch)
   {

   }

}
