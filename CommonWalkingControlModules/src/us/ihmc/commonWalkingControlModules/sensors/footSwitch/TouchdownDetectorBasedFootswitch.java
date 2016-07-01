package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;

import java.util.ArrayList;

public class TouchdownDetectorBasedFootswitch implements FootSwitchInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + "Registry");
   private final ArrayList<TouchdownDetector> touchdownDetectors = new ArrayList<>();

   private final ContactablePlaneBody foot;
   private final double totalRobotWeight;

   private final BooleanYoVariable hasTouchedDown;
   private final YoFramePoint2d yoResolvedCoP;

   public TouchdownDetectorBasedFootswitch(ContactablePlaneBody foot, SDFFullRobotModel fullRobotModel, double totalRobotWeight, YoVariableRegistry parentRegistry)
   {
      this.foot = foot;
      this.totalRobotWeight = totalRobotWeight;

      hasTouchedDown = new BooleanYoVariable("hasTouchedDown", registry);
      yoResolvedCoP = new YoFramePoint2d(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean hasFootHitGround()
   {
      for(TouchdownDetector touchdownDetector : touchdownDetectors)
         touchdownDetector.hasTouchedDown();

      return this.hasTouchedDown.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public void computeAndPackCoP(FramePoint2d copToPack)
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
