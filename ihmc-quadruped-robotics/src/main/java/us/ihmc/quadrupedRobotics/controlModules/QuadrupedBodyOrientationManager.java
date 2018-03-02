package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedBodyOrientationController;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedBodyOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedBodyOrientationController.Setpoints setpoints = new QuadrupedBodyOrientationController.Setpoints();
   private final QuadrupedBodyOrientationController controller;
   private final YoPID3DGains gains;

   private final DoubleParameter[] bodyOrientationProportionalGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] bodyOrientationDerivativeGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] bodyOrientationIntegralGainsParameter = new DoubleParameter[3];
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = new DoubleParameter("bodyOrientationMaxIntegralError", registry,  0);

   private final double[] bodyOrientationProportionalGains = new double[3];
   private final double[] bodyOrientationDerivativeGains = new double[3];
   private final double[] bodyOrientationIntegralGains = new double[3];

   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final FrameQuaternion bodyOrientationReference;
   private final OrientationFrame bodyOrientationReferenceFrame;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   public QuadrupedBodyOrientationManager(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPostureInputProviderInterface postureProvider,
                                          YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.postureProvider = postureProvider;

      for (int i = 0; i < 3; i++)
      {
         bodyOrientationProportionalGainsParameter[i] = new DoubleParameter("bodyOrientationProportionalGain" + Axis.values[i], registry, 5000.0);
         bodyOrientationDerivativeGainsParameter[i] = new DoubleParameter("bodyOrientationDerivativeGain" + Axis.values[i], registry, 750.0);
         bodyOrientationIntegralGainsParameter[i] = new DoubleParameter("bodyOrientationIntegralGain" + Axis.values[i], registry, 0.0);
      }

      controller = new QuadrupedBodyOrientationController(controllerToolbox, registry);
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      gains = controller.getGains();

      bodyOrientationReference = new FrameQuaternion();
      bodyOrientationReferenceFrame = new OrientationFrame(bodyOrientationReference);

      parentRegistry.addChild(registry);
   }

   private void updateGains()
   {
      for (int i = 0; i < 3; i++)
      {
         bodyOrientationProportionalGains[i] = bodyOrientationProportionalGainsParameter[i].getValue();
         bodyOrientationDerivativeGains[i] = bodyOrientationDerivativeGainsParameter[i].getValue();
         bodyOrientationIntegralGains[i] = bodyOrientationIntegralGainsParameter[i].getValue();
      }

      gains.setProportionalGains(bodyOrientationProportionalGains);
      gains.setIntegralGains(bodyOrientationIntegralGains, bodyOrientationMaxIntegralErrorParameter.getValue());
      gains.setDerivativeGains(bodyOrientationDerivativeGains);
   }

   public void initialize(FrameQuaternionReadOnly bodyOrientationEstimate)
   {
      setpoints.initialize(bodyOrientationEstimate);
      controller.reset();
   }

   public void compute(FrameVector3D angularMomentumRateToPack, FrameQuaternionReadOnly bodyOrientationDesired)
   {
      updateGains();

      bodyOrientationReference.setIncludingFrame(bodyOrientationDesired);
      bodyOrientationReference.changeFrame(bodyOrientationReferenceFrame.getParent());
      bodyOrientationReferenceFrame.setOrientationAndUpdate(bodyOrientationReference);

      setpoints.getBodyOrientation().changeFrame(bodyOrientationReferenceFrame);
      setpoints.getBodyOrientation().set(postureProvider.getBodyOrientationInput());
      setpoints.getBodyOrientation().changeFrame(worldFrame);
      double bodyOrientationYaw = setpoints.getBodyOrientation().getYaw();
      double bodyOrientationPitch = setpoints.getBodyOrientation().getPitch() + groundPlaneEstimator.getPitch(bodyOrientationYaw);
      double bodyOrientationRoll = setpoints.getBodyOrientation().getRoll();
      setpoints.getBodyOrientation().setYawPitchRoll(bodyOrientationYaw, bodyOrientationPitch, bodyOrientationRoll);

      setpoints.getBodyAngularVelocity().set(postureProvider.getBodyAngularRateInput());
      setpoints.getComTorqueFeedforward().setToZero();

      controller.compute(angularMomentumRateToPack, setpoints, controllerToolbox.getTaskSpaceEstimates().getBodyAngularVelocity());
   }

}
