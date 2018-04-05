package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
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
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedPostureInputProviderInterface postureProvider;

   private final DivergentComponentOfMotionController dcmPositionController;
//   private final QuadrupedComPositionController comPositionController;
//   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;

   private final QuadrupedForceControllerToolbox controllerToolbox;
   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final double gravity;
   private final double mass;

//   private final ParameterizedPID3DGains comPositionGainsParameter;
   private final DoubleParameter comPositionGravityCompensationParameter = new DoubleParameter("comPositionGravityCompensation", registry, 1);

   private final ReferenceFrame supportFrame;
   private final FramePoint3D cmpPositionSetpoint = new FramePoint3D();

   private final ReferenceFrame centerOfMassFrame;

   private double desiredCoMHeightAcceleration;

   public QuadrupedMomentumRateOfChangeModule(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPostureInputProviderInterface postureProvider,
                                              YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.postureProvider = postureProvider;
      gravity = controllerToolbox.getRuntimeEnvironment().getGravity();
      mass = controllerToolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();

      linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();
      supportFrame = controllerToolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      centerOfMassFrame = controllerToolbox.getReferenceFrames().getCenterOfMassFrame();

      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      ReferenceFrame comFrame = controllerToolbox.getReferenceFrames().getCenterOfMassFrame();
      ReferenceFrame comZUpFrame = controllerToolbox.getReferenceFrames().getCenterOfMassZUpFrame();
      LinearInvertedPendulumModel linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();

      dcmPositionController = new DivergentComponentOfMotionController(comFrame, runtimeEnvironment.getControlDT(), linearInvertedPendulumModel, registry);
//      comPositionController = new QuadrupedComPositionController(comZUpFrame, runtimeEnvironment.getControlDT(), registry);
//      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();

//      DefaultPID3DGains defaultComPositionGains = new DefaultPID3DGains();
//      defaultComPositionGains.setProportionalGains(0.0, 0.0, 5000.0);
//      defaultComPositionGains.setDerivativeGains(0.0, 0.0, 750.0);
//      comPositionGainsParameter = new ParameterizedPID3DGains("_comPosition", GainCoupling.NONE, true, defaultComPositionGains, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      dcmPositionController.reset();
//      comPositionControllerSetpoints.initialize(controllerToolbox.getTaskSpaceEstimates().getComPosition());
//      comPositionController.reset();
   }
   
   public void setDesiredCenterOfMassHeightAcceleration(double desiredCoMHeightAcceleration)
   {
      this.desiredCoMHeightAcceleration = desiredCoMHeightAcceleration;
   }

   public void compute(FrameVector3D linearMomentumRateOfChangeToPack, FixedFramePoint3DBasics vrpPositionSetpointToPack, FixedFramePoint3DBasics cmpPositionSetpointToPack,
                       FramePoint3DReadOnly dcmPositionEstimate, FramePoint3DReadOnly dcmPositionSetpoint,
                       FrameVector3DReadOnly dcmVelocitySetpoint)
   {
//      comPositionController.getGains().set(comPositionGainsParameter);

      dcmPositionController.compute(vrpPositionSetpointToPack, dcmPositionEstimate, dcmPositionSetpoint, dcmVelocitySetpoint);

      double vrpHeightOffsetFromHeightManagement = comPositionGravityCompensationParameter.getValue() * desiredCoMHeightAcceleration * linearInvertedPendulumModel.getComHeight() / gravity;
      vrpPositionSetpointToPack.subZ(vrpHeightOffsetFromHeightManagement);
      cmpPositionSetpoint.set(vrpPositionSetpointToPack);
      cmpPositionSetpoint.subZ(linearInvertedPendulumModel.getComHeight());


      linearInvertedPendulumModel.computeComForce(linearMomentumRateOfChangeToPack, cmpPositionSetpoint);


      cmpPositionSetpoint.changeFrame(cmpPositionSetpointToPack.getReferenceFrame());
      cmpPositionSetpointToPack.set(cmpPositionSetpoint);


      linearMomentumRateOfChangeToPack.changeFrame(worldFrame);
      linearMomentumRateOfChangeToPack.subZ(mass * gravity);


      // update desired com position, velocity, and vertical force
      /*
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(postureProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(postureProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(linearMomentumRateOfChangeToPack);
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.getValue() * mass * gravity);

      comPositionController.compute(linearMomentumRateOfChangeToPack, comPositionControllerSetpoints, controllerToolbox.getTaskSpaceEstimates());*/
   }


   private final FramePoint2D centerOfMass2d = new FramePoint2D();
   private final FrameVector2D achievedCoMAcceleration2d = new FrameVector2D();

   public void computeAchievedCMP(FrameVector3DReadOnly achievedLinearMomentumRate, FixedFramePoint2DBasics achievedCMPToPack)
   {
      if (achievedLinearMomentumRate.containsNaN())
         return;

      centerOfMass2d.setToZero(centerOfMassFrame);
      centerOfMass2d.changeFrame(worldFrame);

      achievedCoMAcceleration2d.setIncludingFrame(achievedLinearMomentumRate);
      achievedCoMAcceleration2d.scale(1.0 / mass);
      achievedCoMAcceleration2d.changeFrame(worldFrame);

      double omega0 = linearInvertedPendulumModel.getNaturalFrequency();

      achievedCMPToPack.set(achievedCoMAcceleration2d);
      achievedCMPToPack.scale(-1.0 / (omega0 * omega0));
      achievedCMPToPack.add(centerOfMass2d);
   }
}
