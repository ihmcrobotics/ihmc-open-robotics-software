package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
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

   private final FramePoint3D eCMPPositionSetpoint = new FramePoint3D();

   private final ReferenceFrame centerOfMassFrame;

   private double desiredCoMHeightAcceleration;

   private final ParameterVector3D linearMomentumRateWeight = new ParameterVector3D("linearMomentumRateWeight", new Vector3D(5.0, 5.0, 2.5), registry);

   private final FrameVector3D linearMomentumRateOfChange = new FrameVector3D();

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();

   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FrameVector3D dcmVelocitySetpoint = new FrameVector3D();

   private final boolean debug;

   public QuadrupedMomentumRateOfChangeModule(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this(controllerToolbox, parentRegistry, false);
   }

   public QuadrupedMomentumRateOfChangeModule(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry, boolean debug)
   {
      this.debug = debug;
      gravity = controllerToolbox.getRuntimeEnvironment().getGravity();
      mass = controllerToolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();

      linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();
      centerOfMassFrame = controllerToolbox.getReferenceFrames().getCenterOfMassFrame();

      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      ReferenceFrame comFrame = controllerToolbox.getReferenceFrames().getCenterOfMassFrame();
      LinearInvertedPendulumModel linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();

      dcmPositionController = new DivergentComponentOfMotionController(controllerToolbox.getSupportPolygons(), comFrame, runtimeEnvironment.getControlDT(),
                                                                       linearInvertedPendulumModel, registry);

      momentumRateCommand.setSelectionMatrixForLinearControl();

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

   public void setDCMEstimate(FramePoint3DReadOnly dcmPositionEstimate)
   {
      this.dcmPositionEstimate.setIncludingFrame(dcmPositionEstimate);
   }

   public void setDCMSetpoints(FramePoint3DReadOnly dcmPositionSetpoint, FrameVector3DReadOnly dcmVelocitySetpoint)
   {
      this.dcmPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
      this.dcmVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);
   }

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();

   public void compute(FixedFramePoint3DBasics vrpPositionSetpointToPack, FixedFramePoint3DBasics eCMPPositionSetpointToPack)
   {
      dcmPositionController.compute(vrpPositionSetpointToPack, dcmPositionEstimate, dcmPositionSetpoint, dcmVelocitySetpoint);

      double vrpHeightOffsetFromHeightManagement = desiredCoMHeightAcceleration * linearInvertedPendulumModel.getLipmHeight() / gravity;
      vrpPositionSetpointToPack.subZ(vrpHeightOffsetFromHeightManagement);

      double modifiedHeight = comPositionGravityCompensationParameter.getValue() * linearInvertedPendulumModel.getLipmHeight();
      eCMPPositionSetpoint.setIncludingFrame(vrpPositionSetpointToPack);
      eCMPPositionSetpoint.subZ(modifiedHeight);
      eCMPPositionSetpointToPack.setMatchingFrame(eCMPPositionSetpoint);

      centerOfMassPosition.setToZero(centerOfMassFrame);
      eCMPPositionSetpoint.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setIncludingFrame(centerOfMassPosition);
      linearMomentumRateOfChange.sub(eCMPPositionSetpoint);
      linearMomentumRateOfChange.scale(mass * MathTools.square(linearInvertedPendulumModel.getNaturalFrequency()));
      linearMomentumRateOfChange.changeFrame(worldFrame);
      linearMomentumRateOfChange.subZ(mass * gravity);

      if (debug && linearMomentumRateOfChange.containsNaN())
         throw new IllegalArgumentException("LinearMomentum rate contains NaN.");

      momentumRateCommand.setLinearMomentumRate(linearMomentumRateOfChange);
      momentumRateCommand.setLinearWeights(linearMomentumRateWeight);
   }

   public FrameVector3DReadOnly getDcmError()
   {
      return dcmPositionController.getDcmError();
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   private final FramePoint3D centerOfMass = new FramePoint3D();
   private final FrameVector3D achievedCoMAcceleration = new FrameVector3D();

   public void computeAchievedECMP(FrameVector3DReadOnly achievedLinearMomentumRate, FixedFramePoint3DBasics achievedECMPToPack)
   {
      if (achievedLinearMomentumRate.containsNaN())
         return;

      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);

      achievedCoMAcceleration.setIncludingFrame(achievedLinearMomentumRate);
      achievedCoMAcceleration.scale(1.0 / mass);
      achievedCoMAcceleration.changeFrame(worldFrame);

      double omega0 = linearInvertedPendulumModel.getNaturalFrequency();

      achievedECMPToPack.set(achievedCoMAcceleration);
      achievedECMPToPack.addZ(gravity); // needs to be an addZ, because gravity in this case is positive.
      achievedECMPToPack.scale(-1.0 / (omega0 * omega0));
      achievedECMPToPack.add(centerOfMass);
   }
}
