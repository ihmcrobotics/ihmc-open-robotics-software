package us.ihmc.robotics.controllers;

import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getAcceleration;
import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getAngle;
import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getAngularVelocity;
import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getRadialVelocity;
import static us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator.getRadius;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class CylindricalCoordinatesPositionController implements PositionController
{
   private final YoVariableRegistry registry;

   private final DoubleYoVariable positionErrorRadial;
   private final DoubleYoVariable positionErrorAngle;
   private final DoubleYoVariable positionErrorZ;

   private final DoubleYoVariable velocityErrorRadial;
   private final DoubleYoVariable velocityErrorAngle;
   private final DoubleYoVariable velocityErrorZ;

   private final DoubleYoVariable kpRadial;
   private final DoubleYoVariable kpAngle;
   private final DoubleYoVariable kpZ;

   private final DoubleYoVariable kdRadial;
   private final DoubleYoVariable kdAngle;
   private final DoubleYoVariable kdZ;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame cylinderFrame;
   private final FramePoint currentPosition;

   public CylindricalCoordinatesPositionController(String prefix, ReferenceFrame bodyFrame, ReferenceFrame cylinderFrame, YoVariableRegistry parentRegistry)
   {
      this.bodyFrame = bodyFrame;
      this.currentPosition = new FramePoint(bodyFrame);
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      positionErrorRadial = new DoubleYoVariable(prefix + "RadialPositionError", registry);
      positionErrorAngle = new DoubleYoVariable(prefix + "AnglePositionError", registry);
      positionErrorZ = new DoubleYoVariable(prefix + "ZPositionError", registry);

      velocityErrorRadial = new DoubleYoVariable(prefix + "RadialVelocityError", registry);
      velocityErrorAngle = new DoubleYoVariable(prefix + "AngleVelocityError", registry);
      velocityErrorZ = new DoubleYoVariable(prefix + "ZVelocityError", registry);

      String baseProportionalGainName = prefix + "Kp";
      kpRadial = new DoubleYoVariable(baseProportionalGainName + "Radial", registry);
      kpAngle = new DoubleYoVariable(baseProportionalGainName + "Angle", registry);
      kpZ = new DoubleYoVariable(baseProportionalGainName + "Z", registry);

      String baseDerivativeGainName = prefix + "Kd";
      kdRadial = new DoubleYoVariable(baseDerivativeGainName + "Radial", registry);
      kdAngle = new DoubleYoVariable(baseDerivativeGainName + "Angle", registry);
      kdZ = new DoubleYoVariable(baseDerivativeGainName + "Z", registry);
      this.cylinderFrame = cylinderFrame;

      parentRegistry.addChild(registry);
   }

   public void compute(FrameVector output, FramePoint desiredPosition, FrameVector desiredVelocity, FrameVector currentVelocity,
                       FrameVector feedForward)
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
