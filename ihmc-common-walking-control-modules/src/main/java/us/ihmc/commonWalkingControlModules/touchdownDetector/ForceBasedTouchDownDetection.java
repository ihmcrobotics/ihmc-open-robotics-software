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
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry ;

   private final YoBoolean isInContact;
   private final DoubleProvider zForceThreshold;
   private final YoDouble measuredZForce;
   private final FrameVector3D footForce = new FrameVector3D();

   private final WrenchCalculator wrenchCalculator;

   public ForceBasedTouchDownDetection(WrenchCalculator wrenchCalculator, RobotQuadrant robotQuadrant, YoVariableRegistry parentRegistry)
   {
      this.wrenchCalculator = wrenchCalculator;
      String prefix = robotQuadrant.getCamelCaseName() + name;
      registry = new YoVariableRegistry(prefix);

      isInContact = new YoBoolean(prefix + "isInContact", registry);
      zForceThreshold = new DoubleParameter(prefix + "zForceThreshold", registry, 40.0);
      measuredZForce = new YoDouble(prefix + "measuredZForce", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean hasTouchedDown()
   {
      return isInContact.getBooleanValue();
   }

   @Override
   public void update()
   {
      footForce.setIncludingFrame(wrenchCalculator.getWrench().getLinearPart());
      footForce.changeFrame(ReferenceFrame.getWorldFrame());
      measuredZForce.set(footForce.getZ());
      isInContact.set(measuredZForce.getDoubleValue() > zForceThreshold.getValue());
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
