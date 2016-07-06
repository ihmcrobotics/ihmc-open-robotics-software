package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootswitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointVelocityBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;

import java.util.ArrayList;

public class QuadrupedTouchdownDetectorBasedFootSwitch extends TouchdownDetectorBasedFootswitch
{
   private final ContactablePlaneBody foot;
   private final SDFFullRobotModel fullRobotModel;
   private final double totalRobotWeight;
   private final YoFramePoint2d yoResolvedCoP;

   public QuadrupedTouchdownDetectorBasedFootSwitch(ContactablePlaneBody foot, SDFFullRobotModel fullRobotModel, double totalRobotWeight, YoVariableRegistry parentRegistry)
   {
      this.foot = foot;
      this.fullRobotModel = fullRobotModel;
      this.totalRobotWeight = totalRobotWeight;
      yoResolvedCoP = new YoFramePoint2d(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);
   }

   @Override
   protected void setupTouchdownDetectors(ArrayList<TouchdownDetector> touchdownDetectors)
   {
      JointVelocityBasedTouchdownDetector jointVelocityBasedTouchdownDetector = new JointVelocityBasedTouchdownDetector(
            fullRobotModel.getOneDoFJointsAsMap().get("front_left_knee_pitch"), registry);

      touchdownDetectors.add(jointVelocityBasedTouchdownDetector);
   }

   @Override
   public boolean hasFootHitGround()
   {
      //TODO possibly something a little fancier with the TouchdownDetectors
      return false;
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
}
