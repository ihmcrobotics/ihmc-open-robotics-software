package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ForceBasedTouchDownDetection implements TouchdownDetector
{
   private final String name = "ForceTchdwnDetect";
   private final YoVariableRegistry registry ;

   private final YoBoolean isInContact;
   private final YoBoolean isDefinitelyInContact;
   private final DoubleProvider zForceThreshold;
   private final DoubleProvider zForceForSureThreshold;
   private final YoDouble measuredZForce;
   private final YoBoolean isTorquingIntoJointLimit;
   private final FrameVector3D footForce = new FrameVector3D();

   private final WrenchCalculator wrenchCalculator;

   private final boolean dontDetectTouchdownIfAtJointLimit;

   public ForceBasedTouchDownDetection(String suffix, WrenchCalculator wrenchCalculator, RobotQuadrant robotQuadrant,
                                       boolean dontDetectTouchdownIfAtJointLimit, DoubleProvider zForceThreshold, DoubleProvider zForceForSureThreshold,
                                       YoVariableRegistry parentRegistry)
   {
      this.wrenchCalculator = wrenchCalculator;
      this.dontDetectTouchdownIfAtJointLimit = dontDetectTouchdownIfAtJointLimit;
      this.zForceThreshold = zForceThreshold;
      this.zForceForSureThreshold = zForceForSureThreshold;
      String prefix = robotQuadrant.getShortName() + name;
      registry = new YoVariableRegistry(prefix);

      isInContact = new YoBoolean(prefix + "IsInContact" + suffix, registry);
      isDefinitelyInContact = new YoBoolean(prefix + "IsDefinitelyInContact" + suffix, registry);

      measuredZForce = new YoDouble(prefix + "MeasuredZForce" + suffix, registry);
      isTorquingIntoJointLimit = new YoBoolean(prefix + "IsTorquingIntoJointLimit" + suffix, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean hasTouchedDown()
   {
      return isInContact.getBooleanValue();
   }

   @Override
   public boolean hasForSureTouchedDown()
   {
      return isDefinitelyInContact.getBooleanValue();
   }

   @Override
   public void update()
   {
      footForce.setIncludingFrame(wrenchCalculator.getWrench().getLinearPart());
      footForce.changeFrame(ReferenceFrame.getWorldFrame());
      measuredZForce.set(footForce.getZ());
      isTorquingIntoJointLimit.set(wrenchCalculator.isTorquingIntoJointLimit());
      if (dontDetectTouchdownIfAtJointLimit && isTorquingIntoJointLimit.getBooleanValue())
      {
         isInContact.set(false);
         isDefinitelyInContact.set(false);
      }
      else
      {
         isInContact.set(measuredZForce.getDoubleValue() > zForceThreshold.getValue());
         isDefinitelyInContact.set(measuredZForce.getDoubleValue() > zForceForSureThreshold.getValue());
      }
   }

   public void reset()
   {
      measuredZForce.set(0.0);
      isInContact.set(false);
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }
}
