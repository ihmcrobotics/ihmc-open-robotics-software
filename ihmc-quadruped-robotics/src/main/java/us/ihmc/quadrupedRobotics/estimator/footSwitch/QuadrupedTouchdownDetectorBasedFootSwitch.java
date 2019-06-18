package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.WrenchCalculator;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class QuadrupedTouchdownDetectorBasedFootSwitch extends TouchdownDetectorBasedFootSwitch
{
   private static final int defaultGlitchWindow = 10;

   private final ContactablePlaneBody foot;
   private final double totalRobotWeight;
   private final YoFramePoint2D yoResolvedCoP;
   private final GlitchFilteredYoBoolean touchdownDetected;
   private final YoBoolean trustTouchdownDetectors;
   private final WrenchCalculator wrenchCalculator;

   private final YoFrameVector3D measuredForce;

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

      measuredForce = new YoFrameVector3D(robotQuadrant.getCamelCaseName() + "_MeasuredForce", null,
                                          ReferenceFrame.getWorldFrame(), registry);
   }

   public YoBoolean getControllerSetFootSwitch()
   {
      return controllerThinksHasTouchedDown;
   }

   public void addTouchdownDetector(TouchdownDetector touchdownDetector)
   {
      necessaryTouchdownDetectors.addTouchdownDetector(touchdownDetector);
   }

   @Override
   public void updateMeasurement()
   {
      wrenchCalculator.calculate();
      measuredForce.setMatchingFrame(wrenchCalculator.getWrench().getLinearPart());
   }

   @Override
   public boolean hasFootHitGround()
   {
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
      measuredForce.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      return Math.abs(measuredForce.getZ()) / totalRobotWeight;
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
      WrenchReadOnly wrench = wrenchCalculator.getWrench();
      if (hasFootHitGround())
         footWrenchToPack.setIncludingFrame(wrenchCalculator.getWrench());
      else
         footWrenchToPack.setToZero(wrench.getReferenceFrame());
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
