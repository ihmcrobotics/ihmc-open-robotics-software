package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
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

   private final ParameterVector3D vmcLinearMomentumRateWeight = new ParameterVector3D("vmcLinearMomentumRateWeight", new Vector3D(5.0, 5.0, 2.5), registry);
   private final ParameterVector3D kinematicsLinearMomentumWeight = new ParameterVector3D("ikLinearMomentumWeight", new Vector3D(100.0, 100.0, 50.0), registry);

   private final FrameVector3D linearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D linearMomentum = new FrameVector3D();

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final MomentumCommand momentumCommand = new MomentumCommand();

   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FrameVector3D dcmVelocitySetpoint = new FrameVector3D();

   private final double controlDT;
   private final boolean debug;

   private final QuadrupedControllerToolbox controllerToolbox;

   public QuadrupedMomentumRateOfChangeModule(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this(controllerToolbox, parentRegistry, false);
   }

   public QuadrupedMomentumRateOfChangeModule(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry, boolean debug)
   {
      this.debug = debug;
      this.controllerToolbox = controllerToolbox;
      gravity = controllerToolbox.getRuntimeEnvironment().getGravity();
      mass = controllerToolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();
      controlDT = controllerToolbox.getRuntimeEnvironment().getControlDT();

      linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();
      centerOfMassFrame = controllerToolbox.getReferenceFrames().getCenterOfMassFrame();

      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      ReferenceFrame comFrame = controllerToolbox.getReferenceFrames().getCenterOfMassFrame();
      LinearInvertedPendulumModel linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();

      dcmPositionController = new DivergentComponentOfMotionController(comFrame, runtimeEnvironment.getControlDT(), linearInvertedPendulumModel, registry);

      momentumRateCommand.setSelectionMatrixForLinearControl();
      momentumCommand.setSelectionMatrixForLinearControl();

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      dcmPositionController.reset();
      linearMomentum.set(controllerToolbox.getCoMVelocityEstimate());
      linearMomentum.scale(mass);
   }

   public void setDesiredCenterOfMassHeightAcceleration(double desiredCoMHeightAcceleration)
   {
      this.desiredCoMHeightAcceleration = desiredCoMHeightAcceleration;
   }

   public void setDCMEstimate(FramePoint3DReadOnly dcmPositionEstimate)
   {
      this.dcmPositionEstimate.setIncludingFrame(dcmPositionEstimate);
   }

   public void setDCMSetpoints(FramePoint3DReadOnly dcmPositionSetpoint, FrameVector3DReadOnly dcmVelocitySetpoint)
   {
      this.dcmPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
      this.dcmVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);
   }

   public void compute(FixedFramePoint3DBasics vrpPositionSetpointToPack, FixedFramePoint3DBasics cmpPositionSetpointToPack)
   {
      dcmPositionController.compute(vrpPositionSetpointToPack, dcmPositionEstimate, dcmPositionSetpoint, dcmVelocitySetpoint);

      double vrpHeightOffsetFromHeightManagement = desiredCoMHeightAcceleration * linearInvertedPendulumModel.getComHeight() / gravity;
      vrpPositionSetpointToPack.subZ(comPositionGravityCompensationParameter.getValue() * vrpHeightOffsetFromHeightManagement);
      cmpPositionSetpoint.set(vrpPositionSetpointToPack);
      cmpPositionSetpoint.subZ(linearInvertedPendulumModel.getComHeight());

      linearInvertedPendulumModel.computeComForce(linearMomentumRateOfChange, cmpPositionSetpoint);

      cmpPositionSetpoint.changeFrame(cmpPositionSetpointToPack.getReferenceFrame());
      cmpPositionSetpointToPack.set(cmpPositionSetpoint);

      linearMomentumRateOfChange.changeFrame(worldFrame);
      linearMomentumRateOfChange.subZ(mass * gravity);

      if (debug && linearMomentumRateOfChange.containsNaN())
         throw new IllegalArgumentException("LinearMomentum rate contains NaN.");

      momentumRateCommand.setLinearMomentumRate(linearMomentumRateOfChange);
      momentumRateCommand.setLinearWeights(vmcLinearMomentumRateWeight);

      linearMomentumRateOfChange.addZ(mass * gravity);
      linearMomentum.scaleAdd(controlDT, linearMomentumRateOfChange, linearMomentum);
      momentumCommand.setLinearMomentum(linearMomentum);
      momentumCommand.setLinearWeights(kinematicsLinearMomentumWeight);
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   public MomentumCommand getMomentumCommand()
   {
      return momentumCommand;
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
