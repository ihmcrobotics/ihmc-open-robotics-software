package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation3DBasics;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.euclid.filters.RateLimitedYoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Computes and integrates the angular velocity of the given frame
 */
public class OrientationOffsetCalculator
{
   private final RateLimitedYoFrameQuaternion orientationOffset;
   private final MovingReferenceFrame referenceFrame;

   private final double integrationDT;
   private final double maxAngle;

   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D rotationVector = new Vector3D();
   private final AxisAngle axisAngle = new AxisAngle();
   private final AxisAngle axisAngleIntegrated = new AxisAngle();

   private final Twist tempTwist = new Twist();
   private final Quaternion tempQuaternion = new Quaternion();

   public OrientationOffsetCalculator(String name, double integrationDT, MovingReferenceFrame referenceFrame, double maxRate, double maxAngle, YoRegistry registry)
   {
      orientationOffset = new RateLimitedYoFrameQuaternion(name, "Offset", registry, maxRate, integrationDT, ReferenceFrame.getWorldFrame());
      this.referenceFrame = referenceFrame;
      this.integrationDT = integrationDT;
      this.maxAngle = maxAngle;
   }

   public void reset()
   {
      axisAngleIntegrated.setToZero();
      orientationOffset.set(axisAngleIntegrated);
   }

   public void integrate()
   {
      referenceFrame.getTwistRelativeToOther(ReferenceFrame.getWorldFrame(), tempTwist);
      tempTwist.changeFrame(referenceFrame);
      angularVelocity.set(tempTwist.getAngularPart());
      rotationVector.setAndScale(integrationDT, angularVelocity);

      axisAngle.setRotationVector(rotationVector);
      axisAngleIntegrated.multiply(axisAngle);

      double angleClamped = EuclidCoreTools.clamp(axisAngleIntegrated.getAngle(), maxAngle);
      axisAngleIntegrated.setAngle(angleClamped);
      tempQuaternion.set(axisAngleIntegrated);
      orientationOffset.update(tempQuaternion);
   }

   public void clear(double alphaLeak)
   {
      double angle = axisAngleIntegrated.getAngle();
      angle *= alphaLeak;
      axisAngleIntegrated.setAngle(angle);

      tempQuaternion.set(axisAngleIntegrated);
      orientationOffset.update(tempQuaternion);
   }

   public FixedFrameOrientation3DBasics getOrientationOffset()
   {
      return orientationOffset;
   }
}