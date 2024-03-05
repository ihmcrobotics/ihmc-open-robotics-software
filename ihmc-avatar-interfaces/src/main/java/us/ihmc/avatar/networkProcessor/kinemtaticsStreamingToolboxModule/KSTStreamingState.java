package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters.InputStateEstimatorType;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input.KSTInputFBControllerStateEstimator;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input.KSTInputFirstOrderStateEstimator;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input.KSTInputStateEstimator;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output.KSTCompiledOutputProcessor;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePose3D;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class KSTStreamingState implements State
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final KSTTools tools;
   private final double toolboxControllerPeriod;

   private final YoDouble timeOfLastMessageSentToController = new YoDouble("timeOfLastMessageSentToController", registry);
   /**
    * Allows to observe when the IK streaming actually publishes its output to the controller.
    */
   private final YoBoolean isPublishing = new YoBoolean("isPublishing", registry);
   private final YoDouble publishingPeriod = new YoDouble("publishingPeriod", registry);
   private final CommandInputManager ikCommandInputManager;

   private final List<KinematicsToolboxOneDoFJointMessage> defaultNeckJointMessages;
   private final KinematicsToolboxRigidBodyMessage defaultPelvisMessage = new KinematicsToolboxRigidBodyMessage();
   private final KinematicsToolboxRigidBodyMessage defaultChestMessage = new KinematicsToolboxRigidBodyMessage();
   private final SideDependentList<List<KinematicsToolboxOneDoFJointMessage>> defaultArmJointMessages = new SideDependentList<>();
   private final RigidBodyBasics head;
   private final OneDoFJointBasics[] neckJoints;
   private final RigidBodyBasics pelvis;
   private final RigidBodyBasics chest;
   private final SideDependentList<RigidBodyBasics> hands = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> armJoints = new SideDependentList<>();
   private final YoDouble lockPoseFilterBreakFrequency = new YoDouble("lockPoseFilterBreakFrequncy", registry);
   private final YoBoolean lockPelvis = new YoBoolean("lockPelvis", registry);
   private final YoBoolean lockChest = new YoBoolean("lockChest", registry);
   private final YoFramePose3D lockPelvisPose = new YoFramePose3D("lockPelvisPose", worldFrame, registry);
   private final AlphaFilteredYoFramePose3D lockPelvisPoseFiltered;
   private final YoFramePose3D lockChestPose = new YoFramePose3D("lockChestPose", worldFrame, registry);
   private final AlphaFilteredYoFramePose3D lockChestPoseFiltered;

   private final YoDouble defaultArmMessageWeight = new YoDouble("defaultArmMessageWeight", registry);
   private final YoDouble defaultNeckMessageWeight = new YoDouble("defaultNeckMessageWeight", registry);
   private final YoVector3D defaultPelvisMessageLinearWeight = new YoVector3D("defaultPelvisMessageLinearWeight", registry);
   private final YoVector3D defaultPelvisMessageAngularWeight = new YoVector3D("defaultPelvisMessageAngularWeight", registry);
   private final YoVector3D defaultChestMessageAngularWeight = new YoVector3D("defaultChestMessageAngularWeight", registry);

   private final YoDouble defaultPelvisMessageLockWeight = new YoDouble("defaultPelvisMessageLockWeight", registry);
   private final YoDouble defaultChestMessageLockWeight = new YoDouble("defaultChestMessageLockWeight", registry);

   //   private final YoDouble preferredArmConfigWeight = new YoDouble("preferredArmConfigWeight", registry);
   //   private final SideDependentList<List<KinematicsToolboxOneDoFJointMessage>> preferredArmJointMessages = new SideDependentList<>();

   private final YoDouble inputWeightDecayFactor;
   private final YoInteger numberOfDecayingInputs = new YoInteger("numberOfDecayingInputs", registry);

   /**
    * Storing when each rigid-body has started being controlled by the user. That allows to identify
    * newly controlled rigid-body for which the weight matrix will be slowly ramped up to smoothly
    * activate the corresponding objective.
    */
   private final Map<RigidBodyBasics, YoDouble> rigidBodyControlStartTimeMap = new HashMap<>();
   private final YoDouble[] rigidBodyControlStartTimeArray;

   /**
    * Buffer of commands used to slowly decay objectives for end-effector that have been discontinued.
    * The decay is done by reducing the objective's weight continuously over time.
    */
   private final RecyclingArrayList<KinematicsToolboxRigidBodyCommand> decayingInputs = new RecyclingArrayList<>(KinematicsToolboxRigidBodyCommand::new);

   private final YoBoolean isStreaming = new YoBoolean("isStreaming", registry);
   private final YoBoolean wasStreaming = new YoBoolean("wasStreaming", registry);
   private final YoDouble linearRateLimit = new YoDouble("linearRateLimit", registry);
   private final YoDouble angularRateLimit = new YoDouble("angularRateLimit", registry);
   private final YoDouble defaultLinearRateLimit = new YoDouble("defaultLinearRateLimit", registry);
   private final YoDouble defaultAngularRateLimit = new YoDouble("defaultAngularRateLimit", registry);

   private final YoDouble defaultLinearWeight = new YoDouble("defaultLinearWeight", registry);
   private final YoDouble defaultAngularWeight = new YoDouble("defaultAngularWeight", registry);

   private final YoDouble streamingBlendingDuration = new YoDouble("streamingBlendingDuration", registry);
   private final KSTCompiledOutputProcessor outputProcessor;

   private final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);
   private final YoDouble timeSinceLastInput = new YoDouble("timeSinceLastInput", registry);
   private final YoDouble rawInputFrequency = new YoDouble("rawInputFrequency", registry);
   private final AlphaFilteredYoVariable inputFrequency;

   private final HumanoidKinematicsToolboxController ikController;
   private final YoPIDSE3Gains ikSolverSpatialGains;
   private final YoPIDGains ikSolverJointGains;

   private final YoEnum<InputStateEstimatorType> activeInputStateEstimator = new YoEnum<>("activeInputStateEstimator", registry, InputStateEstimatorType.class);
   private final Map<InputStateEstimatorType, KSTInputStateEstimator> inputStateEstimatorsMap = new EnumMap<>(InputStateEstimatorType.class);
   private final KSTInputStateEstimator[] inputStateEstimators;

   public KSTStreamingState(KSTTools tools)
   {
      KinematicsStreamingToolboxParameters parameters = tools.getParameters();
      this.tools = tools;
      toolboxControllerPeriod = tools.getToolboxControllerPeriod();
      ikController = tools.getIKController();
      ikSolverSpatialGains = ikController.getDefaultSpatialGains();
      ikSolverJointGains = ikController.getDefaultJointGains();
      ikController.getCenterOfMassSafeMargin().set(parameters.getCenterOfMassSafeMargin());
      ikController.setPublishingSolutionPeriod(parameters.getPublishingSolutionPeriod());
      ikController.getMomentumWeight().set(parameters.getCenterOfMassHoldWeight());
      ikController.minimizeMomentum(parameters.isMinimizeAngularMomentum(), parameters.isMinimizeLinearMomentum());
      ikController.setMomentumWeight(parameters.getAngularMomentumWeight(), parameters.getLinearMomentumWeight());
      FullHumanoidRobotModel desiredFullRobotModel = tools.getDesiredFullRobotModel();
      ikCommandInputManager = tools.getIKCommandInputManager();

      tools.getRegistry().addChild(registry);

      head = desiredFullRobotModel.getHead();
      pelvis = desiredFullRobotModel.getPelvis();
      chest = desiredFullRobotModel.getChest();
      if (head == null)
         neckJoints = new OneDoFJointBasics[0];
      else
         neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);
      defaultNeckJointMessages = Stream.of(neckJoints)
                                       .map(joint -> KinematicsToolboxMessageFactory.newOneDoFJointMessage(joint, 10.0, 0.0))
                                       .collect(Collectors.toList());
      defaultPelvisMessage.setEndEffectorHashCode(pelvis.hashCode());
      defaultPelvisMessage.getDesiredOrientationInWorld().setToZero();
      defaultChestMessage.setEndEffectorHashCode(chest.hashCode());
      defaultChestMessage.getDesiredOrientationInWorld().setToZero();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = desiredFullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         hands.put(robotSide, hand);
         armJoints.put(robotSide, joints);
         defaultArmJointMessages.put(robotSide,
                                     Stream.of(joints)
                                           .map(joint -> KinematicsToolboxMessageFactory.newOneDoFJointMessage(joint, 10.0, 0.0))
                                           .collect(Collectors.toList()));
      }

      defaultArmMessageWeight.set(parameters.getDefaultArmMessageWeight());
      defaultNeckMessageWeight.set(parameters.getDefaultNeckMessageWeight());
      defaultPelvisMessageLinearWeight.set(parameters.getDefaultPelvisMessageLinearWeight());
      defaultPelvisMessageAngularWeight.set(parameters.getDefaultPelvisMessageAngularWeight());
      defaultChestMessageAngularWeight.set(parameters.getDefaultChestMessageAngularWeight());

      defaultPelvisMessageLockWeight.set(parameters.getDefaultPelvisMessageLockWeight());
      defaultChestMessageLockWeight.set(parameters.getDefaultChestMessageLockWeight());

      defaultLinearWeight.set(parameters.getDefaultLinearWeight());
      defaultAngularWeight.set(parameters.getDefaultAngularWeight());
      /*
       * TODO This was introduced to reduce the risk of shoulder flip on Valkyrie, but it seems that it is
       * impacting too much the task-space objectives and preventing the privileged configuration to kick
       * in when there's a singularity.
       */
      //      preferredArmConfigWeight.set(0.075);
      //
      //      for (RobotSide robotSide : RobotSide.values)
      //      {
      //         OneDoFJointBasics[] joints = armJoints.get(robotSide);
      //         preferredArmJointMessages.put(robotSide,
      //                                       Stream.of(joints).map(joint -> KinematicsToolboxMessageFactory.newOneDoFJointMessage(joint, 10.0, 0.0))
      //                                             .collect(Collectors.toList()));
      //      }

      publishingPeriod.set(parameters.getPublishingPeriod());

      defaultLinearRateLimit.set(parameters.getDefaultLinearRateLimit());
      defaultAngularRateLimit.set(parameters.getDefaultAngularRateLimit());
      streamingBlendingDuration.set(parameters.getDefaultStreamingBlendingDuration());
      outputProcessor = new KSTCompiledOutputProcessor(tools, streamingBlendingDuration, isPublishing, registry);

      { // Filter for locking
         DoubleProvider alpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(lockPoseFilterBreakFrequency.getValue(),
                                                                                                      toolboxControllerPeriod);
         lockPoseFilterBreakFrequency.set(0.25);
         lockPelvisPoseFiltered = new AlphaFilteredYoFramePose3D("lockPelvisPoseFiltered", "", lockPelvisPose, alpha, registry);
         lockChestPoseFiltered = new AlphaFilteredYoFramePose3D("lockChestPoseFiltered", "", lockChestPose, alpha, registry);
      }

      YoDouble inputFrequencyAlpha = new YoDouble("inputFrequencyFilter", registry);
      inputFrequencyAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(2.0, toolboxControllerPeriod));
      inputFrequency = new AlphaFilteredYoVariable("inputFrequency", registry, inputFrequencyAlpha, rawInputFrequency);
      inputWeightDecayFactor = new YoDouble("inputDecayFactor", registry);
      inputWeightDecayFactor.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0 / parameters.getInputWeightDecayDuration(),
                                                                                                 toolboxControllerPeriod));

      Collection<? extends RigidBodyBasics> controllableRigidBodies = tools.getIKController().getControllableRigidBodies();
      activeInputStateEstimator.set(parameters.getInputStateEstimatorType());
      inputStateEstimatorsMap.put(InputStateEstimatorType.FIRST_ORDER_LPF,
                                  new KSTInputFirstOrderStateEstimator(controllableRigidBodies, parameters, toolboxControllerPeriod, registry));
      inputStateEstimatorsMap.put(InputStateEstimatorType.FBC_STYLE,
                                  new KSTInputFBControllerStateEstimator(controllableRigidBodies,
                                                                         parameters,
                                                                         toolboxControllerPeriod,
                                                                         () -> 1.0 / inputFrequency.getValue(),
                                                                         registry));
      inputStateEstimators = inputStateEstimatorsMap.values().toArray(new KSTInputStateEstimator[0]);

      for (RigidBodyBasics rigidBody : controllableRigidBodies)
      {
         rigidBodyControlStartTimeMap.put(rigidBody, new YoDouble(rigidBody.getName() + "ControlStartTime", registry));
      }

      rigidBodyControlStartTimeArray = new YoDouble[controllableRigidBodies.size()];

      int index = 0;

      for (RigidBodyBasics rigidBody : controllableRigidBodies)
      {
         rigidBodyControlStartTimeArray[index++] = rigidBodyControlStartTimeMap.get(rigidBody);
      }
   }

   @Override
   public void onEntry()
   {
      isStreaming.set(false);
      wasStreaming.set(false);
      timeOfLastMessageSentToController.set(Double.NEGATIVE_INFINITY);
      ikSolverSpatialGains.setPositionProportionalGains(50.0);
      ikSolverSpatialGains.setOrientationProportionalGains(50.0);
      ikSolverSpatialGains.setPositionMaxFeedbackAndFeedbackRate(linearRateLimit.getValue(), Double.POSITIVE_INFINITY);
      ikSolverSpatialGains.setOrientationMaxFeedbackAndFeedbackRate(angularRateLimit.getValue(), Double.POSITIVE_INFINITY);
      ikSolverJointGains.setKp(50.0);
      ikSolverJointGains.setMaximumFeedbackAndMaximumFeedbackRate(angularRateLimit.getValue(), Double.POSITIVE_INFINITY);
      ikCommandInputManager.submitMessage(tools.getParameters().getDefaultSolverConfiguration());

      /*
       * The desiredFullRobotModel can either be at the current configuration or at a configuration
       * pre-defined at construction. Let's initialize the arms and neck joints using the current robot
       * configuration.
       */
      FullHumanoidRobotModel controllerFullRobotModel = tools.getCurrentFullRobotModel();

      for (OneDoFJointBasics neckJoint : neckJoints)
      {
         neckJoint.setQ(controllerFullRobotModel.getOneDoFJointByName(neckJoint.getName()).getQ());
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         for (OneDoFJointBasics armJoint : armJoints.get(robotSide))
            armJoint.setQ(controllerFullRobotModel.getOneDoFJointByName(armJoint.getName()).getQ());
      }

      for (int i = 0; i < neckJoints.length; i++)
      {
         defaultNeckJointMessages.get(i).setDesiredPosition(controllerFullRobotModel.getOneDoFJointByName(neckJoints[i].getName()).getQ());
         defaultNeckJointMessages.get(i).setWeight(defaultNeckMessageWeight.getValue());
      }

      // TODO change to using mid-feet z-up frame for initializing pelvis and chest
      lockPelvis.set(tools.getConfigurationCommand().isLockPelvis());

      // TODO Make it possible to lock/unlock pelvis/chest while streaming
      if (lockPelvis.getValue())
      {
         lockPelvisPose.setFromReferenceFrame(controllerFullRobotModel.getPelvis().getBodyFixedFrame());
         lockPelvisPoseFiltered.update();
         defaultPelvisMessage.getDesiredPositionInWorld().set(lockPelvisPoseFiltered.getPosition());
         defaultPelvisMessage.getDesiredOrientationInWorld().set(lockPelvisPoseFiltered.getOrientation());
         defaultPelvisMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         defaultPelvisMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         MessageTools.packWeightMatrix3DMessage(defaultPelvisMessageLockWeight.getValue(), defaultPelvisMessage.getLinearWeightMatrix());
         MessageTools.packWeightMatrix3DMessage(defaultPelvisMessageLockWeight.getValue(), defaultPelvisMessage.getAngularWeightMatrix());
      }
      else
      {
         lockPelvisPoseFiltered.reset();
         lockPelvisPose.setFromReferenceFrame(pelvis.getBodyFixedFrame());
         defaultPelvisMessage.getDesiredPositionInWorld().set(lockPelvisPose.getPosition());
         defaultPelvisMessage.getDesiredOrientationInWorld().setToYawOrientation(lockPelvisPose.getYaw());
         defaultPelvisMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, true, worldFrame));
         defaultPelvisMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         MessageTools.packWeightMatrix3DMessage(defaultPelvisMessageLinearWeight, defaultPelvisMessage.getLinearWeightMatrix());
         MessageTools.packWeightMatrix3DMessage(defaultPelvisMessageAngularWeight, defaultPelvisMessage.getAngularWeightMatrix());
      }

      lockChest.set(tools.getConfigurationCommand().isLockChest());

      if (lockChest.getValue())
      {
         lockChestPose.setFromReferenceFrame(controllerFullRobotModel.getChest().getBodyFixedFrame());
         lockChestPoseFiltered.update();
         defaultChestMessage.getDesiredPositionInWorld().set(lockChestPoseFiltered.getPosition());
         defaultChestMessage.getDesiredOrientationInWorld().set(lockChestPoseFiltered.getOrientation());
         defaultChestMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         defaultChestMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         MessageTools.packWeightMatrix3DMessage(defaultChestMessageLockWeight.getValue(), defaultChestMessage.getLinearWeightMatrix());
         MessageTools.packWeightMatrix3DMessage(defaultChestMessageLockWeight.getValue(), defaultChestMessage.getAngularWeightMatrix());
      }
      else
      {
         lockChestPoseFiltered.reset();
         lockChestPose.setFromReferenceFrame(chest.getBodyFixedFrame());
         defaultChestMessage.getDesiredPositionInWorld().setToZero();
         defaultChestMessage.getDesiredOrientationInWorld().setToYawOrientation(lockChestPose.getYaw());
         defaultChestMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, false, worldFrame));
         defaultChestMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         MessageTools.packWeightMatrix3DMessage(0.0, defaultChestMessage.getLinearWeightMatrix());
         MessageTools.packWeightMatrix3DMessage(defaultChestMessageAngularWeight, defaultChestMessage.getAngularWeightMatrix());
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] joints = armJoints.get(robotSide);
         List<KinematicsToolboxOneDoFJointMessage> defaultMessages = defaultArmJointMessages.get(robotSide);

         for (int i = 0; i < joints.length; i++)
         {
            defaultMessages.get(i).setDesiredPosition(controllerFullRobotModel.getOneDoFJointByName(joints[i].getName()).getQ());
            defaultMessages.get(i).setWeight(defaultArmMessageWeight.getValue());
         }
      }

      outputProcessor.initialize();

      timeOfLastInput.set(Double.NaN);
      timeSinceLastInput.set(Double.NaN);
      inputFrequency.reset();

      for (KSTInputStateEstimator inputStateEstimator : inputStateEstimators)
         inputStateEstimator.reset();

      for (YoDouble rigidBodyControlStartTime : rigidBodyControlStartTimeArray)
         rigidBodyControlStartTime.setToNaN();

      System.gc();
   }

   private final KinematicsStreamingToolboxInputCommand filteredInputs = new KinematicsStreamingToolboxInputCommand();
   private final List<RigidBodyBasics> uncontrolledRigidBodies = new ArrayList<>();

   @Override
   public void doAction(double timeInState)
   {
      tools.pollInputCommand();

      FullHumanoidRobotModel controllerFullRobotModel = tools.getCurrentFullRobotModel();

      if (lockPelvis.getValue() && !tools.getConfigurationCommand().isPelvisTaskspaceEnabled())
      {
         lockPelvisPose.setFromReferenceFrame(controllerFullRobotModel.getPelvis().getBodyFixedFrame());
         lockPelvisPoseFiltered.update();
         defaultPelvisMessage.getDesiredPositionInWorld().set(lockPelvisPoseFiltered.getPosition());
         defaultPelvisMessage.getDesiredOrientationInWorld().set(lockPelvisPoseFiltered.getOrientation());
      }
      else
      {
         lockPelvisPoseFiltered.reset();
      }

      if (lockChest.getValue() && !tools.getConfigurationCommand().isChestTaskspaceEnabled())
      {
         lockChestPose.setFromReferenceFrame(controllerFullRobotModel.getChest().getBodyFixedFrame());
         lockChestPoseFiltered.update();
         defaultChestMessage.getDesiredPositionInWorld().set(lockChestPoseFiltered.getPosition());
         defaultChestMessage.getDesiredOrientationInWorld().set(lockChestPoseFiltered.getOrientation());
      }
      else
      {
         lockChestPoseFiltered.reset();
      }

      KinematicsStreamingToolboxInputCommand latestInput = tools.getLatestInput();

      if (latestInput != null)
      {
         isStreaming.set(latestInput.getStreamToController());
         if (latestInput.getStreamInitialBlendDuration() > 0.0)
            streamingBlendingDuration.set(latestInput.getStreamInitialBlendDuration());
         else
            streamingBlendingDuration.set(tools.getParameters().getDefaultStreamingBlendingDuration());

         // Reset the list to keep track of the bodies that are not controlled
         uncontrolledRigidBodies.clear();
         List<? extends RigidBodyBasics> controllableRigidBodies = tools.getIKController().getControllableRigidBodies();

         for (int i = 0; i < controllableRigidBodies.size(); i++)
         {
            uncontrolledRigidBodies.add(controllableRigidBodies.get(i));
         }

         for (int i = 0; i < latestInput.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand rigidBodyInput = latestInput.getInput(i);
            // Sets the user weights only if provided.
            setDefaultWeightIfNeeded(rigidBodyInput.getSelectionMatrix(), rigidBodyInput.getWeightMatrix());

            // Update time for which each rigid body started being controlled.
            YoDouble startTime = rigidBodyControlStartTimeMap.get(rigidBodyInput.getEndEffector());
            if (startTime.isNaN())
               startTime.set(timeInState);

            double controlDuration = timeInState - startTime.getValue();

            if (controlDuration < streamingBlendingDuration.getValue())
            {
               double blendingFactor = MathTools.clamp(controlDuration / streamingBlendingDuration.getValue(), 0.0, 1.0);
               rigidBodyInput.getWeightMatrix().scale(blendingFactor);
            }

            // Update the list of bodies that are not controlled this tick
            uncontrolledRigidBodies.remove(rigidBodyInput.getEndEffector());
         }

         for (int i = 0; i < uncontrolledRigidBodies.size(); i++)
         {
            rigidBodyControlStartTimeMap.get(uncontrolledRigidBodies.get(i)).setToNaN();
         }

         if (!latestInput.getStreamToController() && latestInput.getNumberOfInputs() == 0 && tools.hasPreviousInput())
         {
            /*
             * In case the user abruptly stops streaming and the message is empty.
             * Only then we remember the last inputs and use them to finish this session.
             * Without this, the robot would go to its privileged configuration.
             */
            latestInput.addInputs(tools.getPreviousInput().getInputs());
            filteredInputs.set(latestInput);
         }
         else if (tools.hasNewInputCommand())
         {
            filteredInputs.set(latestInput);
         }

         for (int i = filteredInputs.getNumberOfInputs() - 1; i >= 0; i--)
         { // Removing the inputs for rigid-bodies that are locked.
            RigidBodyBasics endEffector = filteredInputs.getInput(i).getEndEffector();

            if (lockPelvis.getValue() && endEffector == pelvis)
               filteredInputs.removeInput(i);
            else if (lockChest.getValue() && endEffector == chest)
               filteredInputs.removeInput(i);
         }

         for (KSTInputStateEstimator inputStateEstimator : inputStateEstimators)
            inputStateEstimator.update(tools.getTime(), tools.hasNewInputCommand(), filteredInputs, tools.getPreviousInput());

         for (int i = 0; i < filteredInputs.getNumberOfInputs(); i++)
         { // Ship it
            KinematicsToolboxRigidBodyCommand filteredInput = filteredInputs.getInput(i);
            KSTInputStateEstimator inputStateEstimator = inputStateEstimatorsMap.get(activeInputStateEstimator.getValue());
            FramePose3DReadOnly estimatedPose = inputStateEstimator.getEstimatedPose(filteredInput.getEndEffector());
            SpatialVectorReadOnly estimatedVelocity = inputStateEstimator.getEstimatedVelocity(filteredInput.getEndEffector());
            if (estimatedPose != null)
               filteredInput.getDesiredPose().set(estimatedPose);
            if (estimatedVelocity != null)
               filteredInput.getDesiredVelocity().set(estimatedVelocity);
            ikCommandInputManager.submitCommand(filteredInput);
         }

         if (!latestInput.hasInputFor(head))
            ikCommandInputManager.submitMessages(defaultNeckJointMessages);
         if (!latestInput.hasInputFor(pelvis) || lockPelvis.getValue())
            ikCommandInputManager.submitMessage(defaultPelvisMessage);
         if (!latestInput.hasInputFor(chest) || lockChest.getValue())
            ikCommandInputManager.submitMessage(defaultChestMessage);

         for (RobotSide robotSide : RobotSide.values)
         {
            if (!latestInput.hasInputFor(hands.get(robotSide)))
            {
               ikCommandInputManager.submitMessages(defaultArmJointMessages.get(robotSide));
            }
         }

         if (latestInput.getAngularRateLimitation() > 0.0)
            angularRateLimit.set(latestInput.getAngularRateLimitation());
         else
            angularRateLimit.set(defaultAngularRateLimit.getValue());
         if (latestInput.getLinearRateLimitation() > 0.0)
            linearRateLimit.set(latestInput.getLinearRateLimitation());
         else
            linearRateLimit.set(defaultLinearRateLimit.getValue());

         if (tools.hasPreviousInput())
         {
            handleInputsDecay(latestInput, tools.getPreviousInput());
            ikCommandInputManager.submitCommands(decayingInputs);
         }
      }

      if (tools.hasNewInputCommand())
      {
         if (Double.isFinite(timeSinceLastInput.getValue()) && timeSinceLastInput.getValue() > 0.0)
         {
            rawInputFrequency.set(1.0 / timeSinceLastInput.getValue());
            inputFrequency.update();
         }

         timeOfLastInput.set(timeInState);
      }

      if (Double.isFinite(timeOfLastInput.getValue()))
      {
         timeSinceLastInput.set(timeInState - timeOfLastInput.getValue());
      }

      ikSolverSpatialGains.setPositionMaxFeedbackAndFeedbackRate(linearRateLimit.getValue(), Double.POSITIVE_INFINITY);
      ikSolverSpatialGains.setOrientationMaxFeedbackAndFeedbackRate(angularRateLimit.getValue(), Double.POSITIVE_INFINITY);
      ikSolverJointGains.setMaximumFeedbackAndMaximumFeedbackRate(angularRateLimit.getValue(), Double.POSITIVE_INFINITY);
      tools.getIKController().updateInternal();

      /*
       * Updating the desired default position for the arms and/or neck only if the corresponding
       * end-effector is controlled. This allows to improve continuity in case the user stops controlling
       * a hand/head.
       */
      if (latestInput != null)
      {
         if (latestInput.hasInputFor(head))
            KSTTools.copyJointDesiredPositions(neckJoints, defaultNeckJointMessages);

         for (RobotSide robotSide : RobotSide.values)
         {
            if (latestInput.hasInputFor(hands.get(robotSide)))
               KSTTools.copyJointDesiredPositions(armJoints.get(robotSide), defaultArmJointMessages.get(robotSide));
         }
      }

      if (isStreaming.getValue())
      {
         isPublishing.set((timeInState - timeOfLastMessageSentToController.getValue()) >= publishingPeriod.getValue());
         if (isPublishing.getValue())
            timeOfLastMessageSentToController.set(timeInState);
      }
      else if (wasStreaming.getValue())
      {
         isPublishing.set(true);
         timeOfLastMessageSentToController.set(timeInState);
      }
      else
      {
         isPublishing.set(false);
         timeOfLastMessageSentToController.set(Double.NEGATIVE_INFINITY);
      }

      outputProcessor.update(timeInState, wasStreaming.getValue(), isStreaming.getValue(), tools.getIKController().getSolution());

      if (isPublishing.getValue())
      {
         tools.streamToController(outputProcessor.getProcessedOutput(), !isStreaming.getValue());
      }

      wasStreaming.set(isStreaming.getValue());
   }

   private void handleInputsDecay(KinematicsStreamingToolboxInputCommand latestInputs, KinematicsStreamingToolboxInputCommand previousInputs)
   {
      // 1- Forget inputs for end-effectors that are once again being controlled.
      for (int i = decayingInputs.size() - 1; i >= 0; i--)
      {
         KinematicsToolboxRigidBodyCommand decayingInput = decayingInputs.get(i);
         if (latestInputs.hasInputFor(decayingInput.getEndEffector()))
            decayingInputs.remove(i);
      }

      // 2- Register inputs for end-effectors that stopped being controlled.
      for (int i = 0; i < previousInputs.getNumberOfInputs(); i++)
      {
         KinematicsToolboxRigidBodyCommand previousInput = previousInputs.getInput(i);
         if (!latestInputs.hasInputFor(previousInput.getEndEffector()))
         {
            boolean addInputToDecay = true;

            for (int j = 0; j < decayingInputs.size(); j++)
            {
               if (previousInput.getEndEffector() == decayingInputs.get(j).getEndEffector())
               {
                  addInputToDecay = false;
                  break;
               }
            }

            if (addInputToDecay)
               decayingInputs.add().set(previousInput);
         }
      }

      // 3- Decay inputs by reducing their, if it reaches a low threshold then the input is dropped.
      for (int i = decayingInputs.size() - 1; i >= 0; i--)
      {
         KinematicsToolboxRigidBodyCommand decayingInput = decayingInputs.get(i);
         decayingInput.getWeightMatrix().scale(inputWeightDecayFactor.getValue());

         if (findMaximumWeightValue(decayingInput.getWeightMatrix()) < 0.1)
            decayingInputs.remove(i);
      }

      numberOfDecayingInputs.set(decayingInputs.size());
   }

   private static double findMaximumWeightValue(WeightMatrix6D weightMatrix)
   {
      return Math.max(findMaximumWeightValue(weightMatrix.getAngularPart()), findMaximumWeightValue(weightMatrix.getLinearPart()));
   }

   private static double findMaximumWeightValue(WeightMatrix3D weightMatrix)
   {
      return EuclidCoreTools.max(weightMatrix.getX(), weightMatrix.getY(), weightMatrix.getZ());
   }

   private void setDefaultWeightIfNeeded(SelectionMatrix6D selectionMatrix, WeightMatrix6D weightMatrix)
   {
      setDefaultWeightIfNeeded(selectionMatrix.getLinearPart(), weightMatrix.getLinearPart(), defaultLinearWeight.getValue());
      setDefaultWeightIfNeeded(selectionMatrix.getAngularPart(), weightMatrix.getAngularPart(), defaultAngularWeight.getValue());
   }

   private static void setDefaultWeightIfNeeded(SelectionMatrix3D selectionMatrix, WeightMatrix3D weightMatrix, double defaultWeight)
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
   public void onExit(double timeInState)
   {
      tools.flushInputCommands();
   }
}
