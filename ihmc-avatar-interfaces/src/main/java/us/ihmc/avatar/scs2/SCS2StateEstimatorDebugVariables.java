package us.ihmc.avatar.scs2;

import java.util.concurrent.TimeUnit;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CentroidalMomentumRateCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.session.YoTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2StateEstimatorDebugVariables implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoTimer timer = new YoTimer(getClass().getSimpleName(), TimeUnit.MILLISECONDS, registry);

   private final YoFramePoint3D centerOfMassPosition;
   private final YoFrameVector3D centerOfMassVelocity;
   private final YoFrameVector3D centerOfMassAcceleration;

   private final YoFrameVector3D linearMomentum;
   private final YoFrameVector3D angularMomentum;

   private final YoFrameVector3D linearMomentumRate;
   private final YoFrameVector3D angularMomentumRate;

   private final YoFrameVector2D centroidalMomentPivot;

   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final CentroidalMomentumRateCalculator centroidalMomentumRateCalculator;

   private final double mass;
   private final double gravity;

   public SCS2StateEstimatorDebugVariables(ReferenceFrame inertialFrame, double gravity, ControllerInput controllerInput)
   {
      this.gravity = gravity;

      mass = TotalMassCalculator.computeSubTreeMass(controllerInput.getInput().getRootBody());
      centerOfMassFrame = new CenterOfMassReferenceFrame("actualCenterOfMassFrame", inertialFrame, controllerInput.getInput().getRootBody());
      centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(controllerInput.getInput(), centerOfMassFrame);

      centerOfMassPosition = new YoFramePoint3D("actualCenterOfMassPosition", inertialFrame, registry);
      centerOfMassVelocity = new YoFrameVector3D("actualCenterOfMassVelocity", inertialFrame, registry);
      centerOfMassAcceleration = new YoFrameVector3D("actualCenterOfMassAcceleration", inertialFrame, registry);

      linearMomentum = new YoFrameVector3D("actualLinearMomentum", inertialFrame, registry);
      angularMomentum = new YoFrameVector3D("actualAngularMomentum", inertialFrame, registry);

      linearMomentumRate = new YoFrameVector3D("actualLinearMomentumRate", inertialFrame, registry);
      angularMomentumRate = new YoFrameVector3D("actualAngularMomentumRate", inertialFrame, registry);

      centroidalMomentPivot = new YoFrameVector2D("actualCMP", inertialFrame, registry);
   }

   @Override
   public void doControl()
   {
      timer.start();
      centerOfMassFrame.update();
      centroidalMomentumRateCalculator.reset();

      centerOfMassPosition.setFromReferenceFrame(centerOfMassFrame);

      linearMomentum.setMatchingFrame(centroidalMomentumRateCalculator.getMomentum().getLinearPart());
      angularMomentum.setMatchingFrame(centroidalMomentumRateCalculator.getMomentum().getAngularPart());

      centerOfMassVelocity.setAndScale(1.0 / mass, linearMomentum);

      linearMomentumRate.setMatchingFrame(centroidalMomentumRateCalculator.getMomentumRate().getLinearPart());
      angularMomentumRate.setMatchingFrame(centroidalMomentumRateCalculator.getMomentumRate().getAngularPart());

      centerOfMassAcceleration.setAndScale(1.0 / mass, linearMomentumRate);

      // CMP = COMxy - (z/Fz)*Fxy
      centroidalMomentPivot.set(centerOfMassAcceleration);
      double z = centerOfMassPosition.getZ();
      double normalizedFz = -gravity + centerOfMassAcceleration.getZ();
      centroidalMomentPivot.scale(-z / normalizedFz);
      centroidalMomentPivot.addX(centerOfMassPosition.getX());
      centroidalMomentPivot.addY(centerOfMassPosition.getY());
      timer.stop();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
