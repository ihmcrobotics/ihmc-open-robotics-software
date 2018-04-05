package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedMomentumRateOfChangeModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DivergentComponentOfMotionController dcmPositionController;
   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final double gravity;
   private final double mass;

   private final DoubleParameter comPositionGravityCompensationParameter = new DoubleParameter("comPositionGravityCompensation", registry, 1);

   private final FramePoint3D cmpPositionSetpoint = new FramePoint3D();

   private final ReferenceFrame centerOfMassFrame;

   private double desiredCoMHeightAcceleration;

   public QuadrupedMomentumRateOfChangeModule(QuadrupedForceControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      gravity = controllerToolbox.getRuntimeEnvironment().getGravity();
      mass = controllerToolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();

      linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();
      centerOfMassFrame = controllerToolbox.getReferenceFrames().getCenterOfMassFrame();

      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      ReferenceFrame comFrame = controllerToolbox.getReferenceFrames().getCenterOfMassFrame();
      LinearInvertedPendulumModel linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();

      dcmPositionController = new DivergentComponentOfMotionController(comFrame, runtimeEnvironment.getControlDT(), linearInvertedPendulumModel, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      dcmPositionController.reset();
   }
   
   public void setDesiredCenterOfMassHeightAcceleration(double desiredCoMHeightAcceleration)
   {
      this.desiredCoMHeightAcceleration = desiredCoMHeightAcceleration;
   }

   public void compute(FrameVector3D linearMomentumRateOfChangeToPack, FixedFramePoint3DBasics vrpPositionSetpointToPack, FixedFramePoint3DBasics cmpPositionSetpointToPack,
                       FramePoint3DReadOnly dcmPositionEstimate, FramePoint3DReadOnly dcmPositionSetpoint,
                       FrameVector3DReadOnly dcmVelocitySetpoint)
   {
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
