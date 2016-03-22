package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.aware.communication.QuadrupedControllerInputProvider;
import us.ihmc.aware.controller.force.taskSpaceController.*;
import us.ihmc.aware.controller.force.taskSpaceController.controlBlocks.BodyOrientationControlBlock;
import us.ihmc.aware.controller.force.taskSpaceController.controlBlocks.ComPositionControlBlock;
import us.ihmc.aware.controller.force.taskSpaceController.controlBlocks.DcmPositionControlBlock;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVirtualModelBasedStandController implements QuadrupedForceController
{
   private final SDFFullRobotModel fullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final QuadrupedJointNameMap jointNameMap;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final QuadrupedControllerInputProvider inputProvider;

   // parameters
   private final ParameterMap params;
   private final static String BODY_ORIENTATION_PROPORTIONAL_GAINS = "bodyOrientationProportionalGains";
   private final static String BODY_ORIENTATION_DERIVATIVE_GAINS = "bodyOrientationDerivativeGains";
   private final static String BODY_ORIENTATION_INTEGRAL_GAINS = "bodyOrientationIntegralGains";
   private final static String BODY_ORIENTATION_MAX_INTEGRAL_ERROR = "bodyOrientationMaxIntegralError";
   private final static String DCM_PROPORTIONAL_GAINS = "dcmProportionalGains";
   private final static String DCM_INTEGRAL_GAINS = "dcmIntegralGains";
   private final static String DCM_MAX_INTEGRAL_ERROR = "dcmMaxIntegralError";
   private final static String COM_PROPORTIONAL_GAINS = "comProportionalGains";
   private final static String COM_DERIVATIVE_GAINS = "comDerivativeGains";
   private final static String COM_INTEGRAL_GAINS = "comIntegralGains";
   private final static String COM_MAX_INTEGRAL_ERROR = "comMaxIntegralError";

   // frames
   private final QuadrupedReferenceFrames referenceFrames;
   private final PoseReferenceFrame supportFrame;

   // estimates
   QuadrupedSupportPolygon supportPolygonEstimate;
   FramePoint supportCentroidEstimate;
   FrameOrientation supportOrientationEstimate;

   // setpoints
   private final FrameOrientation bodyOrientationSetpoint;
   private final FrameVector bodyAngularVelocitySetpoint;
   private final FramePoint comPositionSetpoint;
   private final FrameVector comVelocitySetpoint;
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;

   // controllers
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceEstimatorSettings taskSpaceEstimatorSettings;
   private final QuadrupedTaskSpaceCommands taskSpaceCommands;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final QuadrupedTaskSpaceControllerSettings taskSpaceControllerSettings;
   private final BodyOrientationControlBlock bodyOrientationControlBlock;
   private final DcmPositionControlBlock dcmPositionControlBlock;
   private final ComPositionControlBlock comPositionControlBlock;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public QuadrupedVirtualModelBasedStandController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters robotParameters, ParameterMapRepository parameterMapRepository, QuadrupedControllerInputProvider inputProvider)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.yoGraphicsListRegistry = runtimeEnvironment.getGraphicsListRegistry();
      this.jointNameMap = robotParameters.getJointMap();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();
      this.inputProvider = inputProvider;

      // parameters
      this.params = parameterMapRepository.get(QuadrupedVirtualModelBasedStandController.class);
      params.setDefault(BODY_ORIENTATION_PROPORTIONAL_GAINS, 5000, 5000, 2500);
      params.setDefault(BODY_ORIENTATION_DERIVATIVE_GAINS, 750, 750, 500);
      params.setDefault(BODY_ORIENTATION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(BODY_ORIENTATION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(DCM_PROPORTIONAL_GAINS, 2, 2, 0);
      params.setDefault(DCM_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(DCM_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(COM_PROPORTIONAL_GAINS, 0, 0, 5000);
      params.setDefault(COM_DERIVATIVE_GAINS, 0, 0, 750);
      params.setDefault(COM_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(COM_MAX_INTEGRAL_ERROR, 0);

      // frames
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, jointNameMap, robotParameters.getPhysicalProperties());
      ReferenceFrame comFrame = referenceFrames.getCenterOfMassZUpFrame();
      ReferenceFrame bodyFrame = referenceFrames.getBodyFrame();
      supportFrame = new PoseReferenceFrame("SupportFrame", ReferenceFrame.getWorldFrame());

      // estimates
      supportPolygonEstimate = new QuadrupedSupportPolygon();
      supportCentroidEstimate = new FramePoint();
      supportOrientationEstimate = new FrameOrientation();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygonEstimate.setFootstep(robotQuadrant, new FramePoint());
      }

      // setpoints
      bodyOrientationSetpoint = new FrameOrientation();
      bodyAngularVelocitySetpoint = new FrameVector();
      comPositionSetpoint = new FramePoint();
      comVelocitySetpoint = new FrameVector();
      dcmPositionSetpoint = new FramePoint();
      dcmVelocitySetpoint = new FrameVector();

      // controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(fullRobotModel, referenceFrames, jointNameMap, registry, yoGraphicsListRegistry);
      taskSpaceEstimatorSettings = new QuadrupedTaskSpaceEstimatorSettings();
      taskSpaceCommands = new QuadrupedTaskSpaceCommands();
      taskSpaceController = new QuadrupedTaskSpaceController(fullRobotModel, referenceFrames, jointNameMap, robotParameters.getQuadrupedJointLimits(), registry, yoGraphicsListRegistry);
      taskSpaceControllerSettings = new QuadrupedTaskSpaceControllerSettings();
      bodyOrientationControlBlock = new BodyOrientationControlBlock(bodyFrame, controlDT, registry);
      dcmPositionControlBlock = new DcmPositionControlBlock(comFrame, controlDT, mass, gravity, registry);
      comPositionControlBlock = new ComPositionControlBlock(comFrame, controlDT, mass, gravity, registry);

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateEstimates()
   {
      // compute support polygon
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceEstimates.getSolePosition().get(robotQuadrant).changeFrame(supportPolygonEstimate.getReferenceFrame());
         supportPolygonEstimate.setFootstep(robotQuadrant, taskSpaceEstimates.getSolePosition().get(robotQuadrant));
         taskSpaceEstimates.getSolePosition().get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
      }
      double minFrontFootHeight = Math.min(taskSpaceEstimates.getSolePosition().get(RobotQuadrant.FRONT_LEFT).getZ(), taskSpaceEstimates.getSolePosition().get(RobotQuadrant.FRONT_RIGHT).getZ());
      double minHindFootHeight = Math.min(taskSpaceEstimates.getSolePosition().get(RobotQuadrant.HIND_LEFT).getZ(), taskSpaceEstimates.getSolePosition().get(RobotQuadrant.HIND_RIGHT).getZ());

      // compute support frame (centroid and nominal orientation)
      supportCentroidEstimate.changeFrame(supportPolygonEstimate.getReferenceFrame());
      supportPolygonEstimate.getCentroid2d(supportCentroidEstimate);
      supportCentroidEstimate.changeFrame(ReferenceFrame.getWorldFrame());
      supportCentroidEstimate.setZ((minFrontFootHeight + minHindFootHeight) / 2.0);
      supportOrientationEstimate.changeFrame(supportPolygonEstimate.getReferenceFrame());
      supportOrientationEstimate.setYawPitchRoll(supportPolygonEstimate.getNominalYaw(), supportPolygonEstimate.getNominalPitch(), 0.0);
      supportFrame.setPoseAndUpdate(supportCentroidEstimate, supportOrientationEstimate);
   }

   private void updateSetpoints()
   {
      // update desired body orientation and angular rate
      bodyOrientationSetpoint.changeFrame(supportFrame);
      bodyOrientationSetpoint.set(inputProvider.getBodyOrientationInput());
      bodyAngularVelocitySetpoint.changeFrame(supportFrame);
      bodyAngularVelocitySetpoint.set(inputProvider.getBodyAngularRateInput());
      bodyOrientationControlBlock.setBodyOrientationSetpoint(bodyOrientationSetpoint);
      bodyOrientationControlBlock.setBodyAngularVelocitySetpoint(bodyAngularVelocitySetpoint);

      // update desired com position and velocity
      comPositionSetpoint.changeFrame(supportFrame);
      comPositionSetpoint.set(inputProvider.getComPositionInput());
      comVelocitySetpoint.changeFrame(supportFrame);
      comVelocitySetpoint.set(inputProvider.getComVelocityInput());
      comPositionControlBlock.setComPositionSetpoint(comPositionSetpoint);

      // update desired dcm position
      dcmPositionSetpoint.changeFrame(supportFrame);
      dcmPositionSetpoint.set(inputProvider.getComVelocityInput());
      dcmPositionSetpoint.scale(1.0 / taskSpaceEstimates.getLipNaturalFrequency());
      dcmPositionSetpoint.add(inputProvider.getComPositionInput());
      dcmVelocitySetpoint.setToZero(supportFrame);
      dcmPositionControlBlock.setDcmPositionSetpoint(dcmPositionSetpoint);
      dcmPositionControlBlock.setDcmVelocitySetpoint(dcmVelocitySetpoint);
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      double comHeightInput = inputProvider.getComPositionInput().getZ();
      taskSpaceEstimatorSettings.setLipNaturalFrequency(Math.sqrt(gravity / Math.max(comHeightInput, 0.001)));
      taskSpaceEstimator.compute(taskSpaceEstimates, taskSpaceCommands, taskSpaceEstimatorSettings);
      updateEstimates();
      updateSetpoints();
      taskSpaceController.compute(taskSpaceEstimates, taskSpaceCommands, taskSpaceControllerSettings);
      return null;
   }

   @Override public void onEntry()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < jointNameMap.getLegJointNames().length; i++)
         {
            // initialize leg joint mode to force control
            LegJointName legJointName = jointNameMap.getLegJointNames()[i];
            String jointName = jointNameMap.getLegJointName(robotQuadrant, legJointName);
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
            joint.setUnderPositionControl(false);
         }
      }

      // initialize task space controller settings
      taskSpaceControllerSettings.setDefaults();
      taskSpaceControllerSettings.setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setSoleForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }

      // initialize task space controller
      taskSpaceController.removeControlBlocks();
      taskSpaceController.addControlBlock(bodyOrientationControlBlock);
      taskSpaceController.addControlBlock(dcmPositionControlBlock);
      taskSpaceController.addControlBlock(comPositionControlBlock);
      taskSpaceController.removeFilterBlocks();
      taskSpaceController.reset();

      // initialize control parameters
      bodyOrientationControlBlock.setControlAxes(1, 1, 1);
      bodyOrientationControlBlock.setProportionalGains(params.getVolatileArray(BODY_ORIENTATION_PROPORTIONAL_GAINS));
      bodyOrientationControlBlock.setIntegralGains(params.getVolatileArray(BODY_ORIENTATION_INTEGRAL_GAINS), params.get(BODY_ORIENTATION_MAX_INTEGRAL_ERROR));
      bodyOrientationControlBlock.setDerivativeGains(params.getVolatileArray(BODY_ORIENTATION_DERIVATIVE_GAINS));
      dcmPositionControlBlock.setControlAxes(1, 1, 0);
      dcmPositionControlBlock.setProportionalGains(params.getVolatileArray(DCM_PROPORTIONAL_GAINS));
      dcmPositionControlBlock.setIntegralGains(params.getVolatileArray(DCM_INTEGRAL_GAINS), params.get(DCM_MAX_INTEGRAL_ERROR));
      comPositionControlBlock.setControlAxes(0, 0, 1);
      comPositionControlBlock.setProportionalGains(params.getVolatileArray(COM_PROPORTIONAL_GAINS));
      comPositionControlBlock.setIntegralGains(params.getVolatileArray(COM_INTEGRAL_GAINS), params.get(COM_MAX_INTEGRAL_ERROR));
      comPositionControlBlock.setDerivativeGains(params.getVolatileArray(COM_DERIVATIVE_GAINS));
   }

   @Override public void onExit()
   {
   }
}
