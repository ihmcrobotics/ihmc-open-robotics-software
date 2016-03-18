package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.aware.communication.QuadrupedControllerInputProvider;
import us.ihmc.aware.controller.force.taskSpaceController.*;
import us.ihmc.aware.controller.force.taskSpaceController.feedbackBlocks.BodyOrientationFeedbackBlock;
import us.ihmc.aware.controller.force.taskSpaceController.feedbackBlocks.ComHeightFeedbackBlock;
import us.ihmc.aware.controller.force.taskSpaceController.feedbackBlocks.DcmHorizontalPositionFeedbackBlock;
import us.ihmc.aware.packets.BodyOrientationPacket;
import us.ihmc.aware.packets.ComPositionPacket;
import us.ihmc.aware.params.ParameterMap;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.vmc.QuadrupedContactForceOptimizationSettings;
import us.ihmc.aware.vmc.QuadrupedVirtualModelControllerSettings;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
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
   private final static String JOINT_DAMPING = "jointDamping";
   private final static String BODY_ORIENTATION_PROPORTIONAL_GAINS = "bodyOrientationProportionalGains";
   private final static String BODY_ORIENTATION_DERIVATIVE_GAINS = "bodyOrientationDerivativeGains";
   private final static String BODY_ORIENTATION_INTEGRAL_GAINS = "bodyOrientationIntegralGains";
   private final static String BODY_ORIENTATION_MAX_INTEGRAL_ERROR = "bodyOrientationMaxIntegralError";
   private final static String DCM_PROPORTIONAL_GAINS = "dcmProportionalGains";
   private final static String DCM_INTEGRAL_GAINS = "dcmIntegralGains";
   private final static String DCM_MAX_INTEGRAL_ERROR = "dcmMaxIntegralError";
   private final static String COM_HEIGHT_PROPORTIONAL_GAIN = "comHeightProportionalGain";
   private final static String COM_HEIGHT_DERIVATIVE_GAIN = "comHeightDerivativeGain";
   private final static String COM_HEIGHT_INTEGRAL_GAIN = "comHeightIntegralGain";
   private final static String COM_HEIGHT_MAX_INTEGRAL_ERROR = "comHeightMaxIntegralError";
   private final static String COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT = "comHeightGravityFeedforwardConstant";

   // frames
   private final QuadrupedReferenceFrames referenceFrames;
   private final PoseReferenceFrame supportFrame;

   // setpoints
   private final FrameOrientation bodyOrientationSetpoint;
   private final FrameVector bodyAngularVelocitySetpoint;
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;

   // controllers
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceEstimatorParameters taskSpaceEstimatorParameters;
   private final QuadrupedTaskSpaceCommands taskSpaceCommands;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final QuadrupedTaskSpaceControllerParameters taskSpaceControllerParameters;
   private final BodyOrientationFeedbackBlock bodyOrientationFeedbackBlock;
   private final DcmHorizontalPositionFeedbackBlock dcmPositionFeedbackBlock;
   private final ComHeightFeedbackBlock comHeightFeedbackBlock;

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
      params.setDefault(JOINT_DAMPING, 1);
      params.setDefault(BODY_ORIENTATION_PROPORTIONAL_GAINS, 5000, 5000, 2500);
      params.setDefault(BODY_ORIENTATION_DERIVATIVE_GAINS, 750, 750, 500);
      params.setDefault(BODY_ORIENTATION_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(BODY_ORIENTATION_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(DCM_PROPORTIONAL_GAINS, 2, 2, 0);
      params.setDefault(DCM_INTEGRAL_GAINS, 0, 0, 0);
      params.setDefault(DCM_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(COM_HEIGHT_PROPORTIONAL_GAIN, 5000);
      params.setDefault(COM_HEIGHT_DERIVATIVE_GAIN, 750);
      params.setDefault(COM_HEIGHT_INTEGRAL_GAIN, 0);
      params.setDefault(COM_HEIGHT_MAX_INTEGRAL_ERROR, 0);
      params.setDefault(COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT, 0.95);

      // frames
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, jointNameMap, robotParameters.getPhysicalProperties());
      supportFrame = new PoseReferenceFrame("SupportFrame", ReferenceFrame.getWorldFrame());
      ReferenceFrame comFrame = referenceFrames.getCenterOfMassZUpFrame();
      ReferenceFrame bodyFrame = referenceFrames.getBodyFrame();

      // setpoints
      bodyOrientationSetpoint = new FrameOrientation();
      bodyAngularVelocitySetpoint = new FrameVector();
      dcmPositionSetpoint = new FramePoint();
      dcmVelocitySetpoint = new FrameVector();

      // controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(fullRobotModel, referenceFrames, jointNameMap, registry);
      taskSpaceEstimatorParameters = new QuadrupedTaskSpaceEstimatorParameters();
      taskSpaceCommands = new QuadrupedTaskSpaceCommands();
      taskSpaceController = new QuadrupedTaskSpaceController(fullRobotModel, referenceFrames, jointNameMap, registry, yoGraphicsListRegistry);
      taskSpaceControllerParameters = new QuadrupedTaskSpaceControllerParameters(robotParameters.getQuadrupedJointLimits());
      bodyOrientationFeedbackBlock = new BodyOrientationFeedbackBlock(bodyFrame, controlDT, registry);
      dcmPositionFeedbackBlock = new DcmHorizontalPositionFeedbackBlock(comFrame, controlDT, mass, gravity, 1.0, registry);
      comHeightFeedbackBlock = new ComHeightFeedbackBlock(controlDT, mass, gravity, registry);

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateEstimates()
   {
      referenceFrames.updateFrames();
      supportFrame.setPoseAndUpdate(taskSpaceEstimates.getSupportCentroid(), taskSpaceEstimates.getSupportOrientation());
   }

   private void updateSetpoints()
   {
      // update desired body orientation
      BodyOrientationPacket bodyOrientationPacket = inputProvider.getBodyOrientationPacket().get();
      double yaw = bodyOrientationPacket.getYaw();
      double pitch = bodyOrientationPacket.getPitch();
      double roll = bodyOrientationPacket.getRoll();
      bodyOrientationSetpoint.changeFrame(supportFrame);
      bodyOrientationSetpoint.setYawPitchRoll(yaw, pitch, roll);
      bodyAngularVelocitySetpoint.setToZero(supportFrame);
      bodyOrientationFeedbackBlock.setBodyOrientationSetpoint(bodyOrientationSetpoint);
      bodyOrientationFeedbackBlock.setBodyAngularVelocitySetpoint(bodyAngularVelocitySetpoint);

      // update desired com height
      ComPositionPacket comPositionPacket = inputProvider.getComPositionPacket().get();
      double comHeight = comPositionPacket.getZ();
      comHeightFeedbackBlock.setComHeightSetpoint(comHeight);

      // update desired dcm position
      double dcmNaturalFrequency = Math.sqrt(gravity / Math.max(comHeight, 0.001));
      dcmPositionSetpoint.setToZero(supportFrame);
      dcmPositionSetpoint.add(0, 0, comHeight);
      dcmVelocitySetpoint.setToZero(supportFrame);
      dcmPositionFeedbackBlock.setDcmNaturalFrequency(dcmNaturalFrequency);
      dcmPositionFeedbackBlock.setDcmPositionSetpoint(dcmPositionSetpoint);
      dcmPositionFeedbackBlock.setDcmVelocitySetpoint(dcmVelocitySetpoint);

      // update estimator parameters
      taskSpaceEstimatorParameters.setDcmNaturalFrequency(dcmNaturalFrequency);
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      updateEstimates();
      taskSpaceEstimator.compute(taskSpaceEstimates, taskSpaceEstimatorParameters);
      updateSetpoints();
      taskSpaceController.compute(taskSpaceEstimates, taskSpaceControllerParameters);
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

      // initialize feedback parameters
      bodyOrientationFeedbackBlock.setProportionalGains(params.getVolatileArray(BODY_ORIENTATION_PROPORTIONAL_GAINS));
      bodyOrientationFeedbackBlock.setIntegralGains(params.getVolatileArray(BODY_ORIENTATION_INTEGRAL_GAINS), params.get(BODY_ORIENTATION_MAX_INTEGRAL_ERROR));
      bodyOrientationFeedbackBlock.setDerivativeGains(params.getVolatileArray(BODY_ORIENTATION_DERIVATIVE_GAINS));
      dcmPositionFeedbackBlock.setProportionalGains(params.getVolatileArray(DCM_PROPORTIONAL_GAINS));
      dcmPositionFeedbackBlock.setIntegralGains(params.getVolatileArray(DCM_INTEGRAL_GAINS), params.get(DCM_MAX_INTEGRAL_ERROR));
      comHeightFeedbackBlock.setProportionalGain(params.get(COM_HEIGHT_PROPORTIONAL_GAIN));
      comHeightFeedbackBlock.setIntegralGain(params.get(COM_HEIGHT_INTEGRAL_GAIN), params.get(COM_HEIGHT_MAX_INTEGRAL_ERROR));
      comHeightFeedbackBlock.setDerivativeGain(params.get(COM_HEIGHT_DERIVATIVE_GAIN));
      comHeightFeedbackBlock.setFeedforwardConstant(params.get(COM_HEIGHT_GRAVITY_FEEDFORWARD_CONSTANT));

      // initialize task space controller
      taskSpaceController.clearFilterBlocks();
      taskSpaceController.clearFeedbackBlocks();
      taskSpaceController.addFeedbackBlock(bodyOrientationFeedbackBlock);
      taskSpaceController.addFeedbackBlock(dcmPositionFeedbackBlock);
      taskSpaceController.addFeedbackBlock(comHeightFeedbackBlock);

      // initialize virtual model controller settings
      QuadrupedVirtualModelControllerSettings virtualModelControllerSettings = taskSpaceControllerParameters.getVirtualModelControllerSettings();
      virtualModelControllerSettings.setJointDamping(params.get(JOINT_DAMPING));

      // initialize contact force optimization settings
      QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings = taskSpaceControllerParameters.getContactForceOptimizationSettings();
      contactForceOptimizationSettings.setDefaults();
      contactForceOptimizationSettings.setComForceCommandWeights(1.0, 1.0, 1.0);
      contactForceOptimizationSettings.setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceOptimizationSettings.setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
      }

      // initialize contact state
      QuadrantDependentList<ContactState> contactState = taskSpaceControllerParameters.getContactState();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
   }

   @Override public void onExit()
   {
   }
}
