package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootswitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.Wrench;

public class QuadrupedTouchdownDetectorBasedFootSwitch extends TouchdownDetectorBasedFootswitch
{
   private final ContactablePlaneBody foot;
   private final double totalRobotWeight;
   private final YoFramePoint2d yoResolvedCoP;
   private final YoBoolean touchdownDetected;
   private final YoBoolean trustTouchdownDetectors;
   private boolean touchdownDetectorsUpdated = false;

   public QuadrupedTouchdownDetectorBasedFootSwitch(RobotQuadrant robotQuadrant, ContactablePlaneBody foot, double totalRobotWeight, YoVariableRegistry parentRegistry)
   {
      super(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch", parentRegistry);

      this.foot = foot;
      this.totalRobotWeight = totalRobotWeight;
      yoResolvedCoP = new YoFramePoint2d(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);
      touchdownDetected = new YoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetected", registry);
      trustTouchdownDetectors = new YoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetectorsTrusted", registry);
   }

   public YoBoolean getControllerSetFootSwitch()
   {
      return controllerThinksHasTouchedDown;
   }

   public void addTouchdownDetector(TouchdownDetector touchdownDetector)
   {
      touchdownDetectors.add(touchdownDetector);
   }

   @Override
   public boolean hasFootHitGround()
   {
      if(!touchdownDetectorsUpdated)
      {
         boolean touchdown = true;
         for (int i = 0; i < touchdownDetectors.size(); i++)
         {
            TouchdownDetector touchdownDetector = touchdownDetectors.get(i);
            touchdownDetector.update();
            touchdown &= touchdownDetector.hasTouchedDown();
         }
         touchdownDetected.set(touchdown);

         touchdownDetectorsUpdated = true;
      }

      if(trustTouchdownDetectors.getBooleanValue())
         return touchdownDetected.getBooleanValue();
      else
         return controllerThinksHasTouchedDown.getBooleanValue();
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
   public void trustFootSwitch(boolean trustFootSwitch)
   {
      this.trustTouchdownDetectors.set(trustFootSwitch);
   }

   @Override
   public void reset()
   {
      touchdownDetectorsUpdated = false;
   }
}
