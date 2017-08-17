package us.ihmc.robotics.controllers;

import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getAcceleration;
import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getAngle;
import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getAngularVelocity;
import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getRadialVelocity;
import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getRadius;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.AngleTools;

public class CylindricalCoordinatesPositionController implements PositionController
{
   private final YoVariableRegistry registry;

   private final YoDouble positionErrorRadial;
   private final YoDouble positionErrorAngle;
   private final YoDouble positionErrorZ;

   private final YoDouble velocityErrorRadial;
   private final YoDouble velocityErrorAngle;
   private final YoDouble velocityErrorZ;

   private final YoDouble kpRadial;
   private final YoDouble kpAngle;
   private final YoDouble kpZ;

   private final YoDouble kdRadial;
   private final YoDouble kdAngle;
   private final YoDouble kdZ;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame cylinderFrame;
   private final FramePoint3D currentPosition;

   public CylindricalCoordinatesPositionController(String prefix, ReferenceFrame bodyFrame, ReferenceFrame cylinderFrame, YoVariableRegistry parentRegistry)
   {
      this.bodyFrame = bodyFrame;
      this.currentPosition = new FramePoint3D(bodyFrame);
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      positionErrorRadial = new YoDouble(prefix + "RadialPositionError", registry);
      positionErrorAngle = new YoDouble(prefix + "AnglePositionError", registry);
      positionErrorZ = new YoDouble(prefix + "ZPositionError", registry);

      velocityErrorRadial = new YoDouble(prefix + "RadialVelocityError", registry);
      velocityErrorAngle = new YoDouble(prefix + "AngleVelocityError", registry);
      velocityErrorZ = new YoDouble(prefix + "ZVelocityError", registry);

      String baseProportionalGainName = prefix + "Kp";
      kpRadial = new YoDouble(baseProportionalGainName + "Radial", registry);
      kpAngle = new YoDouble(baseProportionalGainName + "Angle", registry);
      kpZ = new YoDouble(baseProportionalGainName + "Z", registry);

      String baseDerivativeGainName = prefix + "Kd";
      kdRadial = new YoDouble(baseDerivativeGainName + "Radial", registry);
      kdAngle = new YoDouble(baseDerivativeGainName + "Angle", registry);
      kdZ = new YoDouble(baseDerivativeGainName + "Z", registry);
      this.cylinderFrame = cylinderFrame;

      parentRegistry.addChild(registry);
   }

   public void compute(FrameVector3D output, FramePoint3D desiredPosition, FrameVector3D desiredVelocity, FrameVector3D currentVelocity,
                       FrameVector3D feedForward)
   {
      desiredPosition.changeFrame(cylinderFrame);
      desiredVelocity.changeFrame(cylinderFrame);
      currentVelocity.changeFrame(cylinderFrame);
      feedForward.changeFrame(cylinderFrame);

      currentPosition.setToZero(bodyFrame);
      currentPosition.changeFrame(cylinderFrame);

      double desiredAngle = getAngle(desiredPosition);
      double currentAngle = getAngle(currentPosition);
      double angleError = AngleTools.computeAngleDifferenceMinusPiToPi(desiredAngle, currentAngle);

      double desiredAngularVelocity = getAngularVelocity(currentPosition, desiredVelocity);
      double currentAngularVelocity = getAngularVelocity(currentPosition, currentVelocity);

      double angularVelocityError = desiredAngularVelocity - currentAngularVelocity;

//      if (Math.abs(desiredAngularVelocity) > 1e-11)
//         angleError = Math.copySign(angleError, desiredAngularVelocity);

      double angleFeedBack = kpAngle.getDoubleValue() * angleError + kdAngle.getDoubleValue() * angularVelocityError;

      double desiredRadius = getRadius(desiredPosition);
      double currentRadius = getRadius(currentPosition);
      double radiusError = desiredRadius - currentRadius;

      double desiredRadialVelocity = getRadialVelocity(desiredPosition, desiredVelocity);
      double currentRadialVelocity = getRadialVelocity(currentPosition, currentVelocity);
      double radialVelocityError = desiredRadialVelocity - currentRadialVelocity;

      double radiusFeedBack = kpRadial.getDoubleValue() * radiusError + kdRadial.getDoubleValue() * radialVelocityError;

      double desiredZ = desiredPosition.getZ();
      double currentZ = currentPosition.getZ();
      double zError = desiredZ - currentZ;

      double desiredZVelocity = desiredVelocity.getZ();
      double currentZVelocity = currentVelocity.getZ();
      double zVelocityError = desiredZVelocity - currentZVelocity;

      double zFeedBack = kpZ.getDoubleValue() * zError + kdZ.getDoubleValue() * zVelocityError;

      getAcceleration(output, cylinderFrame, currentAngle, currentAngularVelocity, angleFeedBack, currentRadius, currentRadialVelocity, radiusFeedBack, zFeedBack);
      output.changeFrame(bodyFrame);
      feedForward.changeFrame(bodyFrame);
      output.add(feedForward);


      positionErrorAngle.set(angleError);
      positionErrorRadial.set(radiusError);
      positionErrorZ.set(zError);

      velocityErrorAngle.set(angularVelocityError);
      velocityErrorRadial.set(radialVelocityError);
      velocityErrorZ.set(zVelocityError);
   }

   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   public void setGains(CylindricalPDGains gains)
   {
      this.kpRadial.set(gains.getKpRadius());
      this.kpAngle.set(gains.getKpAngle());
      this.kpZ.set(gains.getKpZ());

      this.kdRadial.set(gains.getKdRadius());
      this.kdAngle.set(gains.getKdAngle());
      this.kdZ.set(gains.getKdZ());
   }
}
