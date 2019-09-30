package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.OutputPublisher;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTStreamingState implements State
{
   private static final double defautlInitialBlendDuration = 2.0;
   private static final double defaultAngularMaxRate = 35.0;
   private static final double defaultLinearMaxRate = 2.0;
   private static final double defaultMessageWeight = 1.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final KSTTools tools;
   private OutputPublisher outputPublisher = m ->
   {
   };
   private final YoDouble timeSinceLastMessageToController;
   private final YoDouble publishingPeriod;
   private final KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommandInputManager ikCommandInputManager;

   private final KinematicsToolboxRigidBodyMessage defaultPelvisMessage = new KinematicsToolboxRigidBodyMessage();
   private final KinematicsToolboxRigidBodyMessage defaultChestMessage = new KinematicsToolboxRigidBodyMessage();
   private final RigidBodyBasics pelvis;
   private final RigidBodyBasics chest;

   private final YoBoolean isStreaming, wasStreaming;
   private final YoBoolean isRateLimiting;
   private final YoDouble linearRateLimit, angularRateLimit;

   private final YoDouble defaultLinearWeight, defaultAngularWeight;

   private final YoDouble streamingStartTime;
   private final YoDouble streamingBlendingDuration;
   private final YoDouble solutionFilterBreakFrequency;
   private KinematicsToolboxOutputStatus lastPublished;
   private final YoKinematicsToolboxOutputStatus ikRobotState, initialRobotState, blendedRobotState, filteredRobotState, fdRobotState;

   private final YoPIDSE3Gains ikSolverGains;

   public KSTStreamingState(KSTTools tools)
   {
      this.tools = tools;
      HumanoidKinematicsToolboxController ikController = tools.getIKController();
      ikSolverGains = ikController.getDefaultGains();
      ikController.getCenterOfMassSafeMargin().set(0.05);
      ikController.getMomentumWeight().set(0.0001);
      desiredFullRobotModel = tools.getDesiredFullRobotModel();
      ikCommandInputManager = tools.getIKCommandInputManager();

      pelvis = desiredFullRobotModel.getPelvis();
      defaultPelvisMessage.setEndEffectorHashCode(pelvis.hashCode());
      defaultPelvisMessage.getDesiredOrientationInWorld().setToZero();
      defaultPelvisMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, true, worldFrame));
      defaultPelvisMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
      defaultPelvisMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(defaultMessageWeight));
      defaultPelvisMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(defaultMessageWeight));
      chest = desiredFullRobotModel.getChest();
      defaultChestMessage.setEndEffectorHashCode(chest.hashCode());
      defaultChestMessage.getDesiredOrientationInWorld().setToZero();
      defaultChestMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, false, worldFrame));
      defaultChestMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
      defaultChestMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(defaultMessageWeight));

      YoVariableRegistry registry = tools.getRegistry();
      defaultLinearWeight = new YoDouble("defaultLinearWeight", registry);
      defaultAngularWeight = new YoDouble("defaultAngularWeight", registry);
      defaultLinearWeight.set(20.0);
      defaultAngularWeight.set(1.0);

      timeSinceLastMessageToController = new YoDouble("timeSinceLastMessageToController", registry);
      publishingPeriod = new YoDouble("publishingPeriod", registry);
      publishingPeriod.set(10.0 * tools.getWalkingControllerPeriod());

      isStreaming = new YoBoolean("isStreaming", registry);
      wasStreaming = new YoBoolean("wasStreaming", registry);
      isRateLimiting = new YoBoolean("isRateLimiting", registry);
      isRateLimiting.set(true);
      linearRateLimit = new YoDouble("linearRateLimit", registry);
      linearRateLimit.set(defaultLinearMaxRate);
      angularRateLimit = new YoDouble("angularRateLimit", registry);
      angularRateLimit.set(defaultAngularMaxRate);

      streamingStartTime = new YoDouble("streamingStartTime", registry);
      streamingBlendingDuration = new YoDouble("streamingBlendingDuration", registry);
      streamingBlendingDuration.set(defautlInitialBlendDuration);
      solutionFilterBreakFrequency = new YoDouble("solutionFilterBreakFrequency", registry);
      solutionFilterBreakFrequency.set(4.0);
      FloatingJointBasics rootJoint = desiredFullRobotModel.getRootJoint();
      OneDoFJointBasics[] oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);
      ikRobotState = new YoKinematicsToolboxOutputStatus("IK", rootJoint, oneDoFJoints, registry);
      initialRobotState = new YoKinematicsToolboxOutputStatus("Initial", rootJoint, oneDoFJoints, registry);
      blendedRobotState = new YoKinematicsToolboxOutputStatus("Blended", rootJoint, oneDoFJoints, registry);
      filteredRobotState = new YoKinematicsToolboxOutputStatus("Filtered", rootJoint, oneDoFJoints, registry);
      fdRobotState = new YoKinematicsToolboxOutputStatus("FD", rootJoint, oneDoFJoints, registry);
   }

   public void setOutputPublisher(OutputPublisher outputPublisher)
   {
      this.outputPublisher = outputPublisher;
   }

   private boolean resetFilter = false;

   @Override
   public void onEntry()
   {
      timeSinceLastMessageToController.set(Double.POSITIVE_INFINITY);
      ikSolverGains.setPositionProportionalGains(50.0);
      ikSolverGains.setOrientationProportionalGains(50.0);
      ikSolverGains.setPositionMaxFeedbackAndFeedbackRate(linearRateLimit.getValue(), Double.POSITIVE_INFINITY);
      ikSolverGains.setOrientationMaxFeedbackAndFeedbackRate(angularRateLimit.getValue(), Double.POSITIVE_INFINITY);
      configurationMessage.setJointVelocityWeight(1.0);
      configurationMessage.setEnableJointVelocityLimits(true);
      ikCommandInputManager.submitMessage(configurationMessage);

      FramePose3D pelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      pelvisPose.changeFrame(worldFrame);
      defaultPelvisMessage.getDesiredPositionInWorld().set(pelvisPose.getPosition());
      defaultPelvisMessage.getDesiredOrientationInWorld().setToYawOrientation(pelvisPose.getYaw());
      FrameQuaternion chestOrientation = new FrameQuaternion(chest.getBodyFixedFrame());
      chestOrientation.changeFrame(worldFrame);
      defaultChestMessage.getDesiredOrientationInWorld().setToYawOrientation(chestOrientation.getYaw());
      resetFilter = true;
      lastPublished = null;
   }

   @Override
   public void doAction(double timeInState)
   {
      KinematicsStreamingToolboxInputCommand latestInput = tools.pollInputCommand();

      if (latestInput != null)
      {
         for (int i = 0; i < latestInput.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = latestInput.getInput(i);
            setDefaultWeightIfNeeded(input.getSelectionMatrix(), input.getWeightMatrix());
            ikCommandInputManager.submitCommand(input);
         }

         if (!latestInput.hasInputFor(pelvis))
            ikCommandInputManager.submitMessage(defaultPelvisMessage);
         if (!latestInput.hasInputFor(chest))
            ikCommandInputManager.submitMessage(defaultChestMessage);

         isStreaming.set(latestInput.getStreamToController());
         if (latestInput.getStreamInitialBlendDuration() > 0.0)
            streamingBlendingDuration.set(latestInput.getStreamInitialBlendDuration());
         else
            streamingBlendingDuration.set(defautlInitialBlendDuration);
         if (latestInput.getAngularRateLimitation() > 0.0)
            angularRateLimit.set(latestInput.getAngularRateLimitation());
         else
            angularRateLimit.set(defaultAngularMaxRate);
         if (latestInput.getLinearRateLimitation() > 0.0)
            linearRateLimit.set(latestInput.getLinearRateLimitation());
         else
            linearRateLimit.set(defaultLinearMaxRate);
      }

      ikSolverGains.setPositionMaxFeedbackAndFeedbackRate(linearRateLimit.getValue(), Double.POSITIVE_INFINITY);
      ikSolverGains.setOrientationMaxFeedbackAndFeedbackRate(angularRateLimit.getValue(), Double.POSITIVE_INFINITY);
      tools.getIKController().updateInternal();
      ikRobotState.set(tools.getIKController().getSolution());

      if (resetFilter)
      {
         filteredRobotState.set(ikRobotState);
         resetFilter = false;
      }
      else
      {
         double alphaFilter = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(solutionFilterBreakFrequency.getValue(),
                                                                                              tools.getToolboxControllerPeriod());
         filteredRobotState.interpolate(ikRobotState.getStatus(), filteredRobotState.getStatus(), alphaFilter);
      }

      if (isStreaming.getValue())
      {
         if (!wasStreaming.getValue())
         {
            tools.getCurrentState(initialRobotState);
            streamingStartTime.set(timeInState);
         }

         double timeInBlending = timeInState - streamingStartTime.getValue();

         timeSinceLastMessageToController.add(tools.getToolboxControllerPeriod());

         if (timeSinceLastMessageToController.getValue() >= publishingPeriod.getValue())
         {
            KinematicsToolboxOutputStatus status;
            fdRobotState.set(filteredRobotState);
            fdRobotState.scaleVelocities(0.50);
//            if (lastPublished != null)
//               fdRobotState.setDesiredVelocitiesByFiniteDifference(lastPublished, filteredRobotState.getStatus(), publishingPeriod.getValue());

            if (timeInBlending < streamingBlendingDuration.getValue())
            {
               double alpha = MathTools.clamp(timeInBlending / streamingBlendingDuration.getValue(), 0.0, 1.0);
               double alphaDot = 1.0 / streamingBlendingDuration.getValue();
               blendedRobotState.interpolate(initialRobotState.getStatus(), fdRobotState.getStatus(), alpha, alphaDot);
               status = blendedRobotState.getStatus();
               outputPublisher.publish(tools.setupWholeBodyTrajectoryMessage(status));
            }
            else
            {
               status = fdRobotState.getStatus();
               outputPublisher.publish(tools.setupWholeBodyTrajectoryMessage(status));
            }

            lastPublished = status;
            timeSinceLastMessageToController.set(0.0);
         }
      }
      else
      {
         timeSinceLastMessageToController.set(Double.POSITIVE_INFINITY);
      }

      wasStreaming.set(isStreaming.getValue());
   }

   private void setDefaultWeightIfNeeded(SelectionMatrix6D selectionMatrix, WeightMatrix6D weightMatrix)
   {
      setDefaultWeightIfNeeded(selectionMatrix.getLinearPart(), weightMatrix.getLinearPart(), defaultLinearWeight.getValue());
      setDefaultWeightIfNeeded(selectionMatrix.getAngularPart(), weightMatrix.getAngularPart(), defaultAngularWeight.getValue());
   }

   private void setDefaultWeightIfNeeded(SelectionMatrix3D selectionMatrix, WeightMatrix3D weightMatrix, double defaultWeight)
   {
      if (selectionMatrix.isXSelected())
      {
         if (Double.isNaN(weightMatrix.getX()) || weightMatrix.getX() <= 0.0)
            weightMatrix.setXAxisWeight(defaultWeight);
      }
      if (selectionMatrix.isYSelected())
      {
         if (Double.isNaN(weightMatrix.getY()) || weightMatrix.getY() <= 0.0)
            weightMatrix.setYAxisWeight(defaultWeight);
      }
      if (selectionMatrix.isZSelected())
      {
         if (Double.isNaN(weightMatrix.getZ()) || weightMatrix.getZ() <= 0.0)
            weightMatrix.setZAxisWeight(defaultWeight);
      }
   }

   @Override
   public void onExit()
   {
   }
}
