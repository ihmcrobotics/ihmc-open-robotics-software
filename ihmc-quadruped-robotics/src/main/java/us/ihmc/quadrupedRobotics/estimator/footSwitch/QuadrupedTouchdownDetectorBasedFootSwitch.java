package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.WrenchCalculator;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
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
   private final YoBoolean trustTouchdownDetectorsInSwing;
   private final YoBoolean trustTouchdownDetectorsInSupport;
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
      trustTouchdownDetectorsInSwing = new YoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetectorsTrustedInSwing", registry);
      trustTouchdownDetectorsInSupport= new YoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetectorsTrustedInSupport", registry);

      measuredForce = new YoFrameVector3D(robotQuadrant.getCamelCaseName() + "_MeasuredForce", "",
                                          ReferenceFrame.getWorldFrame(), registry);
   }

   public void addTouchdownDetector(TouchdownDetector touchdownDetector)
   {
      touchdownDetectors.add(touchdownDetector);
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
      for (int i = 0; i < touchdownDetectors.size(); i++)
         touchdownDetectors.get(i).update();

      boolean hasTouchedDown = true;
      for (int i = 0; i < touchdownDetectors.size(); i++)
      {
         if (touchdownDetectors.get(i).hasForSureTouchedDown())
         {
            break;
         }
         else if (!touchdownDetectors.get(i).hasTouchedDown())
         {
            hasTouchedDown = false;
            break;
         }
      }

      touchdownDetected.update(hasTouchedDown);

      boolean thinksInSupport = controllerThinksHasTouchedDown.getBooleanValue();
      if (thinksInSupport && trustTouchdownDetectorsInSupport.getBooleanValue())
         return touchdownDetected.getBooleanValue();
      else if (!thinksInSupport && trustTouchdownDetectorsInSwing.getBooleanValue())
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
   public void trustFootSwitchInSwing(boolean trustFootSwitch)
   {
      this.trustTouchdownDetectorsInSwing.set(trustFootSwitch);
   }

   @Override
   public void trustFootSwitchInSupport(boolean trustFootSwitch)
   {
      this.trustTouchdownDetectorsInSupport.set(trustFootSwitch);
   }

   @Override
   public void reset()
   {
      for (int i = 0; i < touchdownDetectors.size(); i++)
         touchdownDetectors.get(i).reset();

      touchdownDetected.set(false);
   }


}
