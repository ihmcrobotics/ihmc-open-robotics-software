package us.ihmc.avatar.scs2;

import java.util.concurrent.TimeUnit;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.session.YoFixedReferenceFrameUsingYawPitchRoll;
import us.ihmc.scs2.session.YoTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2StateEstimatorDebugVariables implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoTimer timer = new YoTimer(getClass().getSimpleName(), TimeUnit.MILLISECONDS, registry);

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final YoFramePoint3D centerOfMassPosition;
   private final YoFrameVector3D centerOfMassVelocity;
   private final YoFrameVector3D centerOfMassAcceleration;

   private final YoFrameVector3D linearMomentum;
   private final YoFrameVector3D angularMomentum;
   private final YoFrameVector3D linearMomentumPrevious;
   private final YoFrameVector3D angularMomentumPrevious;

   private final YoFrameVector3D linearMomentumRate;
   private final YoFrameVector3D angularMomentumRate;

   private final YoFrameVector2D centroidalMomentPivot;

   private final YoFixedReferenceFrameUsingYawPitchRoll centerOfMassFrame;
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;

   private final double mass;
   private final double gravity;
   private final double dt;

   public SCS2StateEstimatorDebugVariables(ReferenceFrame inertialFrame, double gravity, double dt, ControllerInput controllerInput)
   {
      this.gravity = gravity;
      this.dt = dt;

      centerOfMassCalculator = new CenterOfMassCalculator(controllerInput.getInput().getRootBody(), inertialFrame);
      mass = centerOfMassCalculator.getTotalMass();
      centerOfMassFrame = new YoFixedReferenceFrameUsingYawPitchRoll("centerOfMassFrame", "actualCenterOfMass", inertialFrame, registry);
      centroidalMomentumCalculator = new CentroidalMomentumCalculator(controllerInput.getInput(), centerOfMassFrame);

      centerOfMassPosition = centerOfMassFrame.getOffset().getPosition();
      centerOfMassVelocity = new YoFrameVector3D("actualCenterOfMassVelocity", inertialFrame, registry);
      centerOfMassAcceleration = new YoFrameVector3D("actualCenterOfMassAcceleration", inertialFrame, registry);

      linearMomentum = new YoFrameVector3D("actualLinearMomentum", inertialFrame, registry);
      angularMomentum = new YoFrameVector3D("actualAngularMomentum", inertialFrame, registry);
      linearMomentumPrevious = new YoFrameVector3D("actualLinearMomentumPrevious", inertialFrame, registry);
      angularMomentumPrevious = new YoFrameVector3D("actualAngularMomentumPrevious", inertialFrame, registry);

      linearMomentumRate = new YoFrameVector3D("actualLinearMomentumRate", inertialFrame, registry);
      angularMomentumRate = new YoFrameVector3D("actualAngularMomentumRate", inertialFrame, registry);

      centroidalMomentPivot = new YoFrameVector2D("actualCMP", inertialFrame, registry);
   }

   @Override
   public void doControl()
   {
      timer.start();
      centerOfMassCalculator.reset();
      centerOfMassPosition.set(centerOfMassCalculator.getCenterOfMass());
      centerOfMassFrame.update();
      centroidalMomentumCalculator.reset();

      linearMomentumPrevious.set(linearMomentum);
      angularMomentumPrevious.set(angularMomentum);

      MomentumReadOnly momentum = centroidalMomentumCalculator.getMomentum();
      linearMomentum.setMatchingFrame(momentum.getLinearPart());
      angularMomentum.setMatchingFrame(momentum.getAngularPart());

      centerOfMassVelocity.setAndScale(1.0 / mass, linearMomentum);

      linearMomentumRate.sub(linearMomentum, linearMomentumPrevious);
      linearMomentumRate.scale(1.0 / dt);
      angularMomentumRate.sub(angularMomentum, angularMomentumPrevious);
      angularMomentumRate.scale(1.0 / dt);

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
