package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.Axis;
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
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.yoVariables.parameters.DoubleParameter;
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

   private final ParameterizedPID3DGains comPositionGainsParameter;
   private final DoubleParameter comPositionGravityCompensationParameter = new DoubleParameter("comPositionGravityCompensation", registry, 1);

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

      DefaultPID3DGains defaultComPositionGains = new DefaultPID3DGains();
      defaultComPositionGains.setProportionalGains(0.0, 0.0, 5000.0);
      defaultComPositionGains.setDerivativeGains(0.0, 0.0, 750.0);
      comPositionGainsParameter = new ParameterizedPID3DGains("_comPosition", GainCoupling.NONE, true, defaultComPositionGains, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      dcmPositionController.reset();
      comPositionControllerSetpoints.initialize(controllerToolbox.getTaskSpaceEstimates().getComPosition());
      comPositionController.reset();
   }

   public void compute(FrameVector3D linearMomentumRateOfChangeToPack, FixedFramePoint3DBasics vrpPositionSetpointToPack, FixedFramePoint3DBasics cmpPositionSetpointToPack,
                       FramePoint3DReadOnly dcmPositionEstimate, FramePoint3DReadOnly dcmPositionSetpoint,
                       FrameVector3DReadOnly dcmVelocitySetpoint)
   {
      comPositionController.getGains().set(comPositionGainsParameter);

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
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.getValue() * mass * gravity);

      comPositionController.compute(linearMomentumRateOfChangeToPack, comPositionControllerSetpoints, controllerToolbox.getTaskSpaceEstimates());
   }
}
