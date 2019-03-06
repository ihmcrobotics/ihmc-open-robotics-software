package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.WrenchCalculator;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class QuadrupedTouchdownDetectorBasedFootSwitch extends TouchdownDetectorBasedFootSwitch
{
   private static final int defaultGlitchWindow = 10;

   private final ContactablePlaneBody foot;
   private final double totalRobotWeight;
   private final YoFramePoint2D yoResolvedCoP;
   private final GlitchFilteredYoBoolean touchdownDetected;
   private final YoBoolean trustTouchdownDetectors;
   private final WrenchCalculator wrenchCalculator;

   private final YoFixedFrameWrench measuredWrench;

   public QuadrupedTouchdownDetectorBasedFootSwitch(RobotQuadrant robotQuadrant, ContactablePlaneBody foot, WrenchCalculator wrenchCalculator,
                                                    double totalRobotWeight, YoVariableRegistry parentRegistry)
   {
      this(robotQuadrant, foot, wrenchCalculator, defaultGlitchWindow, totalRobotWeight, parentRegistry);
   }

   public QuadrupedTouchdownDetectorBasedFootSwitch(RobotQuadrant robotQuadrant, ContactablePlaneBody foot, WrenchCalculator wrenchCalculator,
                                                    int glitchWindow, double totalRobotWeight, YoVariableRegistry parentRegistry)
   {
      super(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch", parentRegistry);

      this.foot = foot;
      this.wrenchCalculator = wrenchCalculator;
      this.totalRobotWeight = totalRobotWeight;
      yoResolvedCoP = new YoFramePoint2D(foot.getName() + "ResolvedCoP", "", foot.getSoleFrame(), registry);
      touchdownDetected = new GlitchFilteredYoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetected", registry, glitchWindow);
      trustTouchdownDetectors = new YoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetectorsTrusted", registry);

      measuredWrench = new YoFixedFrameWrench(robotQuadrant.getCamelCaseName() + "_MeasuredWrench", foot.getRigidBody().getBodyFixedFrame(),
                                              foot.getSoleFrame(), registry);
   }

   public YoBoolean getControllerSetFootSwitch()
   {
      return controllerThinksHasTouchedDown;
   }

   public void addTouchdownDetector(TouchdownDetector touchdownDetector)
   {
      necessaryTouchdownDetectors.addTouchdownDetector(touchdownDetector);
   }

   private void updateMeasurement()
   {
      wrenchCalculator.calculate();
      measuredWrench.setMatchingFrame(wrenchCalculator.getWrench());
   }

   @Override
   public boolean hasFootHitGround()
   {
      updateMeasurement();

      necessaryTouchdownDetectors.update();
      touchdownDetected.update(necessaryTouchdownDetectors.hasTouchedDown());

      if(trustTouchdownDetectors.getBooleanValue())
         return touchdownDetected.getBooleanValue();
      else
         return controllerThinksHasTouchedDown.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      updateMeasurement();
      return Math.abs(measuredWrench.getLinearPartZ()) / totalRobotWeight;
   }

   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
   {
      copToPack.setToZero(getMeasurementFrame());
   }

   @Override
   public void updateCoP()
   {
      yoResolvedCoP.setToZero();
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setIncludingFrame(wrenchCalculator.getWrench());
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
      necessaryTouchdownDetectors.reset();
      touchdownDetected.set(false);
   }
}
