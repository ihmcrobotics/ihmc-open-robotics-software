package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootVelocityBasedTouchDownDetection implements TouchdownDetector
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry ;

   private final MovingReferenceFrame soleFrame;

   private final YoBoolean isInContact;
   private final DoubleProvider speedThreshold;
   private final YoDouble measuredSpeed;
   private final FrameVector3D footVelocity = new FrameVector3D();

   public FootVelocityBasedTouchDownDetection(MovingReferenceFrame soleFrame, RobotQuadrant robotQuadrant, YoVariableRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;
      String prefix = robotQuadrant.getCamelCaseName() + name;
      registry = new YoVariableRegistry(prefix);

      isInContact = new YoBoolean(prefix + "IsInContact", registry);
      speedThreshold = new DoubleParameter(prefix + "FootSpeedThreshold", registry, 0.1);
      measuredSpeed = new YoDouble(prefix + "MeasuredSpeed", registry);

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
      footVelocity.setMatchingFrame(soleFrame.getTwistOfFrame().getLinearPart());
      footVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      measuredSpeed.set(footVelocity.length());
      isInContact.set(measuredSpeed.getDoubleValue() < speedThreshold.getValue());
   }

   public void reset()
   {
      measuredSpeed.set(0.0);
      isInContact.set(false);
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }
}
