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
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class FootVelocityBasedTouchDownDetection implements TouchdownDetector
{
   private final String name = "VelTchdwnDetect";
   private final YoVariableRegistry registry ;

   private final MovingReferenceFrame soleFrame;

   private final YoBoolean isInContact;
   private final DoubleProvider speedThreshold;
   private final DoubleProvider zVelocityThreshold;
   private final YoDouble measuredSpeed;
   private final YoFrameVector3D footVelocity;

   public FootVelocityBasedTouchDownDetection(MovingReferenceFrame soleFrame, RobotQuadrant robotQuadrant, YoVariableRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;
      String prefix = robotQuadrant.getShortName() + name;
      registry = new YoVariableRegistry(prefix);

      isInContact = new YoBoolean(prefix + "IsInContact", registry);
      speedThreshold = new DoubleParameter(prefix + "FootSpeedThreshold", registry, 0.8);
      zVelocityThreshold = new DoubleParameter(prefix + "FootZVelocityThreshold", registry, 0.2);
      footVelocity = new YoFrameVector3D(prefix + "FootVelocity", ReferenceFrame.getWorldFrame(), registry);
      measuredSpeed = new YoDouble(prefix + "MeasuredSpeed", registry);

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
      return false;
   }

   @Override
   public void update()
   {
      footVelocity.setMatchingFrame(soleFrame.getTwistOfFrame().getLinearPart());
      measuredSpeed.set(footVelocity.length());
      boolean totalSpeedLowEnough = measuredSpeed.getDoubleValue() < speedThreshold.getValue();
      boolean zVelocitySlowEnough = Math.abs(footVelocity.getZ()) < zVelocityThreshold.getValue();
      isInContact.set(totalSpeedLowEnough && zVelocitySlowEnough);
   }

   public void reset()
   {
      measuredSpeed.set(0.0);
      isInContact.set(false);
   }

   @Override
   public String getName()
   {
      return name;
   }
}
