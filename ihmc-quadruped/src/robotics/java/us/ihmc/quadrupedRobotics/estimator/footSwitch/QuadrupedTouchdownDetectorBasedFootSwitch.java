package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.TouchdownDetectorBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.WrenchCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedTouchdownDetectorBasedFootSwitch extends TouchdownDetectorBasedFootSwitch implements QuadrupedFootSwitchInterface
{

   private static final int defaultGlitchWindow = 10;

   private final IntegerProvider glitchWindowSize;
   private final ContactablePlaneBody foot;
   private final double totalRobotWeight;
   private final GlitchFilteredYoBoolean touchdownDetected;
   private final YoBoolean trustTouchdownDetectorsInSwing;
   private final YoBoolean trustTouchdownDetectorsInSupport;
   private final WrenchCalculator wrenchCalculator;

   private final YoFrameVector3D measuredForce;
   private final YoDouble footLoadPercentage;

   public QuadrupedTouchdownDetectorBasedFootSwitch(String variableSuffix,
                                                    RobotQuadrant robotQuadrant,
                                                    ContactablePlaneBody foot,
                                                    WrenchCalculator wrenchCalculator,
                                                    IntegerProvider glitchWindowSize,
                                                    double totalRobotWeight,
                                                    YoRegistry parentRegistry)
   {
      super(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch" + variableSuffix, parentRegistry);

      this.foot = foot;
      this.wrenchCalculator = wrenchCalculator;
      this.glitchWindowSize = glitchWindowSize;
      this.totalRobotWeight = totalRobotWeight;
      touchdownDetected = new GlitchFilteredYoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetected" + variableSuffix, registry, defaultGlitchWindow);
      trustTouchdownDetectorsInSwing = new YoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetectorsTrustedInSwing" + variableSuffix, registry);
      trustTouchdownDetectorsInSupport = new YoBoolean(robotQuadrant.getCamelCaseName() + "TouchdownDetectorsTrustedInSupport" + variableSuffix, registry);
      footLoadPercentage = new YoDouble(robotQuadrant.getCamelCaseName() + "FootLoadPercentage" + variableSuffix, registry);

      measuredForce = new YoFrameVector3D(robotQuadrant.getCamelCaseName() + "_MeasuredForce", variableSuffix, ReferenceFrame.getWorldFrame(), registry);
   }

   public void addTouchdownDetector(TouchdownDetector touchdownDetector)
   {
      touchdownDetectors.add(touchdownDetector);
   }

   @Override
   public void update()
   {
      touchdownDetected.setWindowSize(glitchWindowSize.getValue());

      wrenchCalculator.calculate();
      measuredForce.setMatchingFrame(wrenchCalculator.getWrench().getLinearPart());

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

      footLoadPercentage.set(Math.max(measuredForce.getZ(), 0.0) / totalRobotWeight);
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      // switching to lift off
      if (controllerThinksHasTouchedDown.getBooleanValue() && !hasFootHitGround)
         touchdownDetected.set(false);

      super.setFootContactState(hasFootHitGround);
   }

   @Override
   public boolean hasFootHitGroundSensitive()
   {
      boolean thinksInSupport = controllerThinksHasTouchedDown.getBooleanValue();
      if (thinksInSupport && trustTouchdownDetectorsInSupport.getBooleanValue())
         return touchdownDetected.getBooleanValue();
      else if (!thinksInSupport && trustTouchdownDetectorsInSwing.getBooleanValue())
         return touchdownDetected.getBooleanValue();
      else
         return controllerThinksHasTouchedDown.getBooleanValue();
   }

   @Override
   public double getFootLoadPercentage()
   {
      return footLoadPercentage.getDoubleValue();
   }

   @Override
   public FramePoint2DReadOnly getCenterOfPressure()
   {
      return null;
   }

   private final Wrench footWrench = new Wrench();

   @Override
   public WrenchReadOnly getMeasuredWrench()
   {
      WrenchReadOnly wrench = wrenchCalculator.getWrench();
      if (hasFootHitGroundFiltered())
         footWrench.setIncludingFrame(wrenchCalculator.getWrench());
      else
         footWrench.setToZero(wrench.getReferenceFrame());
      return footWrench;
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
