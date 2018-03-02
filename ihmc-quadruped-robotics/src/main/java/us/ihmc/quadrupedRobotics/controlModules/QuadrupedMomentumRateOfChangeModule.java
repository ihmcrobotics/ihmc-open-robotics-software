package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedComPositionController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedMomentumRateOfChangeModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedPostureInputProviderInterface postureProvider;

   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;

   private final QuadrupedForceControllerToolbox controllerToolbox;
   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final double gravity;
   private final double mass;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleParameter comPositionGravityCompensationParameter = parameterFactory.createDouble("comPositionGravityCompensation", 1);

   private final ReferenceFrame supportFrame;
   private final FramePoint3D cmpPositionSetpoint = new FramePoint3D();

   public QuadrupedMomentumRateOfChangeModule(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPostureInputProviderInterface postureProvider,
                                              YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.postureProvider = postureProvider;
      gravity = controllerToolbox.getRuntimeEnvironment().getGravity();
      mass = controllerToolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();

      linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();
      supportFrame = controllerToolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      ReferenceFrame comZUpFrame = controllerToolbox.getReferenceFrames().getCenterOfMassZUpFrame();
      LinearInvertedPendulumModel linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();

      dcmPositionController = new DivergentComponentOfMotionController(comZUpFrame, runtimeEnvironment.getControlDT(), linearInvertedPendulumModel, registry);
      comPositionController = new QuadrupedComPositionController(comZUpFrame, runtimeEnvironment.getControlDT(), registry);
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      dcmPositionController.reset();
      comPositionControllerSetpoints.initialize(controllerToolbox.getTaskSpaceEstimates().getComPosition());
      comPositionController.reset();
   }

   private void updateGains()
   {
      comPositionController.getGains().setProportionalGains(comPositionProportionalGainsParameter.get());
      comPositionController.getGains().setIntegralGains(comPositionIntegralGainsParameter.get(), comPositionMaxIntegralErrorParameter.get());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGainsParameter.get());
   }

   public void compute(FrameVector3D linearMomentumRateOfChangeToPack, FixedFramePoint3DBasics vrpPositionSetpointToPack, FixedFramePoint3DBasics cmpPositionSetpointToPack,
                       FramePoint3DReadOnly dcmPositionEstimate, FramePoint3DReadOnly dcmPositionSetpoint,
                       FrameVector3DReadOnly dcmVelocitySetpoint)
   {
      updateGains();

      dcmPositionController.compute(vrpPositionSetpointToPack, dcmPositionEstimate, dcmPositionSetpoint, dcmVelocitySetpoint);

      cmpPositionSetpoint.set(vrpPositionSetpointToPack);
      cmpPositionSetpoint.subZ(linearInvertedPendulumModel.getComHeight());
      linearInvertedPendulumModel.computeComForce(linearMomentumRateOfChangeToPack, cmpPositionSetpoint);

      cmpPositionSetpoint.changeFrame(cmpPositionSetpointToPack.getReferenceFrame());
      cmpPositionSetpointToPack.set(cmpPositionSetpoint);

      linearMomentumRateOfChangeToPack.changeFrame(supportFrame);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(postureProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(postureProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(linearMomentumRateOfChangeToPack);
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.get() * mass * gravity);

      comPositionController.compute(linearMomentumRateOfChangeToPack, comPositionControllerSetpoints, controllerToolbox.getTaskSpaceEstimates());
   }
}
