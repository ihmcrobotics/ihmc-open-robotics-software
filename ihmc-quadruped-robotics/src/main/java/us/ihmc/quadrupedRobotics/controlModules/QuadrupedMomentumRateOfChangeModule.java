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

   private final DoubleParameter[] comPositionProportionalGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] comPositionDerivativeGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] comPositionIntegralGainsParameter = new DoubleParameter[3];
   private final DoubleParameter comPositionMaxIntegralErrorParameter = new DoubleParameter("comPositionMaxIntegralError", registry, 0);
   private final DoubleParameter comPositionGravityCompensationParameter = new DoubleParameter("comPositionGravityCompensation", registry, 1);

   private final double[] comPositionProportionalGains = new double[3];
   private final double[] comPositionDerivativeGains = new double[3];
   private final double[] comPositionIntegralGains = new double[3];

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

      for (int i = 0; i < 3; i++)
      {
         double proportionalGain = (i == 2) ? 5000.0 : 0.0;
         double derivativeGain = (i == 2) ? 750.0 : 0.0;
         comPositionProportionalGainsParameter[i] = new DoubleParameter("comPositionProportionalGain" + Axis.values[i], registry, proportionalGain);
         comPositionDerivativeGainsParameter[i] = new DoubleParameter("comPositionDerivativeGain" + Axis.values[i], registry, derivativeGain);
         comPositionIntegralGainsParameter[i] = new DoubleParameter("comPositionIntegralGain" + Axis.values[i], registry, 0.0);
      }

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
      for (int i = 0; i < 3; i++)
      {
         comPositionProportionalGains[i] = comPositionProportionalGainsParameter[i].getValue();
         comPositionDerivativeGains[i] = comPositionDerivativeGainsParameter[i].getValue();
         comPositionIntegralGains[i] = comPositionIntegralGainsParameter[i].getValue();
      }

      comPositionController.getGains().setProportionalGains(comPositionProportionalGains);
      comPositionController.getGains().setIntegralGains(comPositionIntegralGains, comPositionMaxIntegralErrorParameter.getValue());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGains);
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
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.getValue() * mass * gravity);

      comPositionController.compute(linearMomentumRateOfChangeToPack, comPositionControllerSetpoints, controllerToolbox.getTaskSpaceEstimates());
   }
}
