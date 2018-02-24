package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedBodyOrientationController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedBodyOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedBodyOrientationController.Setpoints setpoints = new QuadrupedBodyOrientationController.Setpoints();
   private final QuadrupedBodyOrientationController controller;
   private final YoPID3DGains gains;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory.createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory.createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);

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

      controller = new QuadrupedBodyOrientationController(controllerToolbox, registry);
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      gains = controller.getGains();

      bodyOrientationReference = new FrameQuaternion();
      bodyOrientationReferenceFrame = new OrientationFrame(bodyOrientationReference);

      parentRegistry.addChild(registry);
   }

   private void updateGains()
   {
      gains.setProportionalGains(bodyOrientationProportionalGainsParameter.get());
      gains.setIntegralGains(bodyOrientationIntegralGainsParameter.get(), bodyOrientationMaxIntegralErrorParameter.get());
      gains.setDerivativeGains(bodyOrientationDerivativeGainsParameter.get());
   }

   public void initialize(QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      setpoints.initialize(taskSpaceEstimates);
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
