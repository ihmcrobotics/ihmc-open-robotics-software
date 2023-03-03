package us.ihmc.avatar.scs2;

import java.util.concurrent.TimeUnit;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.session.YoFixedReferenceFrameUsingYawPitchRoll;
import us.ihmc.scs2.session.YoTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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

   private final YoDouble actualAccelerationFilterBreakFrequency;
   private final AlphaFilteredYoFrameVector centerOfMassAccelerationFiltered;
   private final AlphaFilteredYoFrameVector linearMomentumRateFiltered;
   private final AlphaFilteredYoFrameVector angularMomentumRateFiltered;
   private final AlphaFilteredYoFrameVector2d centroidalMomentPivotFiltered;
   private final AlphaFilteredYoFrameVector centerOfMassAccelerationFiltered2;
   private final AlphaFilteredYoFrameVector linearMomentumRateFiltered2;
   private final AlphaFilteredYoFrameVector angularMomentumRateFiltered2;
   private final AlphaFilteredYoFrameVector2d centroidalMomentPivotFiltered2;

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

      actualAccelerationFilterBreakFrequency = new YoDouble("actualAccelerationFilterBreakFrequency", registry);
      actualAccelerationFilterBreakFrequency.set(0.05 / dt); // 50Hz when running at 1kHz
      DoubleProvider alphaProvider = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(actualAccelerationFilterBreakFrequency.getValue(),
                                                                                                           dt);
      centerOfMassAccelerationFiltered = new AlphaFilteredYoFrameVector("actualCenterOfMassAccelerationFiltered",
                                                                        "",
                                                                        registry,
                                                                        alphaProvider,
                                                                        centerOfMassAcceleration);
      linearMomentumRateFiltered = new AlphaFilteredYoFrameVector("actualLinearMomentumRateFiltered", "", registry, alphaProvider, linearMomentumRate);
      angularMomentumRateFiltered = new AlphaFilteredYoFrameVector("actualAngularMomentumRateFiltered", "", registry, alphaProvider, angularMomentumRate);
      centroidalMomentPivotFiltered = new AlphaFilteredYoFrameVector2d("actualCMPFiltered", "", registry, alphaProvider, centroidalMomentPivot);

      centerOfMassAccelerationFiltered2 = new AlphaFilteredYoFrameVector("actualCenterOfMassAccelerationFiltered2",
                                                                         "",
                                                                         registry,
                                                                         alphaProvider,
                                                                         centerOfMassAccelerationFiltered);
      linearMomentumRateFiltered2 = new AlphaFilteredYoFrameVector("actualLinearMomentumRateFiltered2",
                                                                   "",
                                                                   registry,
                                                                   alphaProvider,
                                                                   linearMomentumRateFiltered);
      angularMomentumRateFiltered2 = new AlphaFilteredYoFrameVector("actualAngularMomentumRateFiltered2",
                                                                    "",
                                                                    registry,
                                                                    alphaProvider,
                                                                    angularMomentumRateFiltered);
      centroidalMomentPivotFiltered2 = new AlphaFilteredYoFrameVector2d("actualCMPFiltered2", "", registry, alphaProvider, centroidalMomentPivotFiltered);
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

      centerOfMassAccelerationFiltered.update();
      linearMomentumRateFiltered.update();
      angularMomentumRateFiltered.update();

      centerOfMassAccelerationFiltered2.update();
      linearMomentumRateFiltered2.update();
      angularMomentumRateFiltered2.update();

      // CMP = COMxy - (z/Fz)*Fxy
      centroidalMomentPivot.set(centerOfMassAcceleration);
      double z = centerOfMassPosition.getZ();
      double normalizedFz = -gravity + centerOfMassAcceleration.getZ();
      if (EuclidCoreTools.isZero(normalizedFz, 1.0e-6))
      {
         centroidalMomentPivot.setToNaN();
         centroidalMomentPivotFiltered.setToNaN();
         centroidalMomentPivotFiltered.reset();
         centroidalMomentPivotFiltered2.setToNaN();
         centroidalMomentPivotFiltered2.reset();
      }
      else
      {
         centroidalMomentPivot.scale(-z / normalizedFz);
         centroidalMomentPivot.addX(centerOfMassPosition.getX());
         centroidalMomentPivot.addY(centerOfMassPosition.getY());

         centroidalMomentPivotFiltered.update();
         centroidalMomentPivotFiltered2.update();
      }
      timer.stop();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
