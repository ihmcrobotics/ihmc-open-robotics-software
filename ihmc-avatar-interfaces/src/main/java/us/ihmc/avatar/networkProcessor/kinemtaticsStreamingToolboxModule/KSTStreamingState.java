package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.WholeBodyStreamingMessagePublisher;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.WholeBodyTrajectoryMessagePublisher;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.tools.UnitConversions;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class KSTStreamingState implements State
{
   private static final double defautlInitialBlendDuration = 2.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final KSTTools tools;
   private final double toolboxControllerPeriod;

   private final YoBoolean useStreamingPublisher = new YoBoolean("useStreamingPublisher", registry);

   private WholeBodyTrajectoryMessagePublisher trajectoryMessagePublisher = m ->
   {
   };
   private WholeBodyStreamingMessagePublisher streamingMessagePublisher = null;
   private final YoDouble timeOfLastMessageSentToController = new YoDouble("timeOfLastMessageSentToController", registry);
   private final YoDouble publishingPeriod = new YoDouble("publishingPeriod", registry);
   private final KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
   private final FullHumanoidRobotModel desiredFullRobotModel;
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
   private final YoBoolean lockPelvis = new YoBoolean("lockPelvis", registry);
   private final YoBoolean lockChest = new YoBoolean("lockChest", registry);
   private final YoFramePose3D lockPelvisPose = new YoFramePose3D("lockPelvisPose", worldFrame, registry);
   private final YoFramePose3D lockChestPose = new YoFramePose3D("lockChestPose", worldFrame, registry);

   private final YoDouble defaultArmMessageWeight = new YoDouble("defaultArmMessageWeight", registry);
   private final YoDouble defaultNeckMessageWeight = new YoDouble("defaultNeckMessageWeight", registry);
   private final YoDouble defaultPelvisMessageLinearWeight = new YoDouble("defaultPelvisMessageLinearWeight", registry);
   private final YoDouble defaultPelvisMessageAngularWeight = new YoDouble("defaultPelvisMessageAngularWeight", registry);
   private final YoDouble defaultChestMessageAngularWeight = new YoDouble("defaultChestMessageAngularWeight", registry);

   private final YoDouble defaultPelvisMessageLockWeight = new YoDouble("defaultPelvisMessageLockWeight", registry);
   private final YoDouble defaultChestMessageLockWeight = new YoDouble("defaultChestMessageLockWeight", registry);

   //   private final YoDouble preferredArmConfigWeight = new YoDouble("preferredArmConfigWeight", registry);
   //   private final SideDependentList<List<KinematicsToolboxOneDoFJointMessage>> preferredArmJointMessages = new SideDependentList<>();

   private final YoDouble inputDecayFactor;
   private final YoInteger numberOfDecayingInputs = new YoInteger("numberOfDecayingInputs", registry);
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

   private final YoDouble streamingStartTime = new YoDouble("streamingStartTime", registry);
   private final YoDouble streamingBlendingDuration = new YoDouble("streamingBlendingDuration", registry);
   private final YoDouble solutionFilterBreakFrequency = new YoDouble("solutionFilterBreakFrequency", registry);
   private final YoKinematicsToolboxOutputStatus ikRobotState, initialRobotState, blendedRobotState, filteredRobotState, outputRobotState;
   private final YoDouble outputJointVelocityScale = new YoDouble("outputJointVelocityScale", registry);

   private final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);
   private final YoDouble timeSinceLastInput = new YoDouble("timeSinceLastInput", registry);
   private final YoDouble rawInputFrequency = new YoDouble("rawInputFrequency", registry);
   private final AlphaFilteredYoVariable inputFrequency;
   private final YoDouble inputsFilterBreakFrequency = new YoDouble("inputsFilterBreakFrequency", registry);

   private final HumanoidKinematicsToolboxController ikController;
   private final YoPIDSE3Gains ikSolverSpatialGains;
   private final YoPIDGains ikSolverJointGains;

   private final Map<RigidBodyBasics, YoFramePoint3D> rawInputPositionMap = new HashMap<>();
   private final Map<RigidBodyBasics, YoFrameQuaternion> rawInputOrientationMap = new HashMap<>();
   private final Map<RigidBodyBasics, YoFramePoint3D> rawExtrapolatedInputPositionMap = new HashMap<>();
   private final Map<RigidBodyBasics, YoFrameQuaternion> rawExtrapolatedInputOrientationMap = new HashMap<>();
   private final Map<RigidBodyBasics, AlphaFilteredYoFramePoint> filteredExtrapolatedInputPositionMap = new HashMap<>();
   private final Map<RigidBodyBasics, AlphaFilteredYoFrameQuaternion> filteredExtrapolatedInputOrientationMap = new HashMap<>();
   private final YoFixedFrameSpatialVector[] rawInputSpatialVelocityArray;
   private final Map<RigidBodyBasics, YoFixedFrameSpatialVector> rawInputSpatialVelocityMap = new HashMap<>();
   private final YoFixedFrameSpatialVector[] decayingInputSpatialVelocityArray;
   private final Map<RigidBodyBasics, YoFixedFrameSpatialVector> decayingInputSpatialVelocityMap = new HashMap<>();
   private final YoDouble inputVelocityDecayFactor = new YoDouble("inputVelocityDecayFactor", registry);
   private final YoDouble inputVelocityDecayDuration = new YoDouble("inputVelocityDecayDuration", registry);

   public KSTStreamingState(KSTTools tools)
   {
      useStreamingPublisher.set(true);
      this.tools = tools;
      toolboxControllerPeriod = tools.getToolboxControllerPeriod();
      ikController = tools.getIKController();
      ikSolverSpatialGains = ikController.getDefaultSpatialGains();
      ikSolverJointGains = ikController.getDefaultJointGains();
      ikController.getCenterOfMassSafeMargin().set(0.05);
      ikController.setPublishingSolutionPeriod(UnitConversions.hertzToSeconds(60.0));
      desiredFullRobotModel = tools.getDesiredFullRobotModel();
      ikCommandInputManager = tools.getIKCommandInputManager();

      tools.getRegistry().addChild(registry);

      head = desiredFullRobotModel.getHead();
      pelvis = desiredFullRobotModel.getPelvis();
      chest = desiredFullRobotModel.getChest();
      if (head == null)
         neckJoints = new OneDoFJointBasics[0];
      else
         neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);
      defaultNeckJointMessages = Stream.of(neckJoints).map(joint -> KinematicsToolboxMessageFactory.newOneDoFJointMessage(joint, 10.0, 0.0))
                                       .collect(Collectors.toList());
      defaultPelvisMessage.setEndEffectorHashCode(pelvis.hashCode());
      defaultPelvisMessage.getDesiredOrientationInWorld().setToZero();
      defaultChestMessage.setEndEffectorHashCode(chest.hashCode());
      defaultChestMessage.getDesiredOrientationInWorld().setToZero();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = desiredFullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] joints;
         if (hand == null)
            joints = new OneDoFJointBasics[0];
         else
            joints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         hands.put(robotSide, hand);
         armJoints.put(robotSide, joints);
         defaultArmJointMessages.put(robotSide,
                                     Stream.of(joints).map(joint -> KinematicsToolboxMessageFactory.newOneDoFJointMessage(joint, 10.0, 0.0))
                                           .collect(Collectors.toList()));
      }

      defaultArmMessageWeight.set(10.0);
      defaultNeckMessageWeight.set(10.0);
      defaultPelvisMessageLinearWeight.set(2.5);
      defaultPelvisMessageAngularWeight.set(1.0);
      defaultChestMessageAngularWeight.set(0.75);

      defaultPelvisMessageLockWeight.set(1000.0);
      defaultChestMessageLockWeight.set(1000.0);

      defaultLinearWeight.set(20.0);
      defaultAngularWeight.set(1.0);
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

      publishingPeriod.set(5.0 * tools.getWalkingControllerPeriod());

      defaultLinearRateLimit.set(1.5);
      defaultAngularRateLimit.set(10.0);
      outputJointVelocityScale.set(0.75);

      streamingBlendingDuration.set(defautlInitialBlendDuration);
      solutionFilterBreakFrequency.set(Double.POSITIVE_INFINITY);
      FloatingJointBasics rootJoint = desiredFullRobotModel.getRootJoint();
      OneDoFJointBasics[] oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);
      ikRobotState = new YoKinematicsToolboxOutputStatus("IK", rootJoint, oneDoFJoints, registry);
      initialRobotState = new YoKinematicsToolboxOutputStatus("Initial", rootJoint, oneDoFJoints, registry);
      blendedRobotState = new YoKinematicsToolboxOutputStatus("Blended", rootJoint, oneDoFJoints, registry);
      filteredRobotState = new YoKinematicsToolboxOutputStatus("Filtered", rootJoint, oneDoFJoints, registry);
      outputRobotState = new YoKinematicsToolboxOutputStatus("FD", rootJoint, oneDoFJoints, registry);

      YoDouble inputFrequencyAlpha = new YoDouble("inputFrequencyFilter", registry);
      inputFrequencyAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(2.0, toolboxControllerPeriod));
      inputFrequency = new AlphaFilteredYoVariable("inputFrequency", registry, inputFrequencyAlpha, rawInputFrequency);
      inputDecayFactor = new YoDouble("inputDecayFactor", registry);
      inputDecayFactor.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0 / 3.0, toolboxControllerPeriod));

      Collection<? extends RigidBodyBasics> controllableRigidBodies = tools.getIKController().getControllableRigidBodies();
      DoubleProvider inputsAlphaProvider = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(inputsFilterBreakFrequency.getValue(),
                                                                                                                 toolboxControllerPeriod);
      inputsFilterBreakFrequency.set(2.0);
      inputVelocityDecayDuration.set(0.5);

      for (RigidBodyBasics rigidBody : controllableRigidBodies)
      {
         String namePrefix = rigidBody.getName() + "Input";
         YoFramePoint3D rawInputPosition;
         YoFrameQuaternion rawInputOrientation;
         YoFramePoint3D rawExtrapolatedInputPosition;
         YoFrameQuaternion rawExtrapolatedInputOrientation;
         AlphaFilteredYoFramePoint filteredExtrapolatedInputPosition;
         AlphaFilteredYoFrameQuaternion filteredExtrapolatedInputOrientation;

         rawInputPosition = new YoFramePoint3D(namePrefix + "RawPosition", worldFrame, registry);
         rawInputOrientation = new YoFrameQuaternion(namePrefix + "RawOrientation", worldFrame, registry);
         rawExtrapolatedInputPosition = new YoFramePoint3D(namePrefix + "RawExtrapolatedPosition", worldFrame, registry);
         rawExtrapolatedInputOrientation = new YoFrameQuaternion(namePrefix + "RawExtrapolatedOrientation", worldFrame, registry);
         filteredExtrapolatedInputPosition = new AlphaFilteredYoFramePoint(namePrefix
               + "FilteredExtrapolatedPosition", "", registry, inputsAlphaProvider, rawExtrapolatedInputPosition);
         filteredExtrapolatedInputOrientation = new AlphaFilteredYoFrameQuaternion(namePrefix
               + "FilteredExtrapolatedOrientation", "", rawExtrapolatedInputOrientation, inputsAlphaProvider, registry);
         YoFixedFrameSpatialVector rawInputSpatialVelocity = new YoFixedFrameSpatialVector(namePrefix + "RawVelocity", worldFrame, registry);
         YoFixedFrameSpatialVector decayingInputSpatialVelocity = new YoFixedFrameSpatialVector(namePrefix + "DecayingVelocity", worldFrame, registry);
         rawInputPositionMap.put(rigidBody, rawInputPosition);
         rawInputOrientationMap.put(rigidBody, rawInputOrientation);
         rawExtrapolatedInputPositionMap.put(rigidBody, rawExtrapolatedInputPosition);
         rawExtrapolatedInputOrientationMap.put(rigidBody, rawExtrapolatedInputOrientation);
         filteredExtrapolatedInputPositionMap.put(rigidBody, filteredExtrapolatedInputPosition);
         filteredExtrapolatedInputOrientationMap.put(rigidBody, filteredExtrapolatedInputOrientation);
         rawInputSpatialVelocityMap.put(rigidBody, rawInputSpatialVelocity);
         decayingInputSpatialVelocityMap.put(rigidBody, decayingInputSpatialVelocity);
      }

      rawInputSpatialVelocityArray = new YoFixedFrameSpatialVector[controllableRigidBodies.size()];
      decayingInputSpatialVelocityArray = new YoFixedFrameSpatialVector[controllableRigidBodies.size()];

      int index = 0;

      for (RigidBodyBasics rigidBody : controllableRigidBodies)
      {
         rawInputSpatialVelocityArray[index] = rawInputSpatialVelocityMap.get(rigidBody);
         decayingInputSpatialVelocityArray[index] = decayingInputSpatialVelocityMap.get(rigidBody);
         index++;
      }
   }

   public void setTrajectoryMessagerPublisher(WholeBodyTrajectoryMessagePublisher outputPublisher)
   {
      this.trajectoryMessagePublisher = outputPublisher;
   }

   public void setStreamingMessagePublisher(WholeBodyStreamingMessagePublisher streamingMessagePublisher)
   {
      this.streamingMessagePublisher = streamingMessagePublisher;
   }

   private boolean resetFilter = false;

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
      configurationMessage.setJointVelocityWeight(1.0);
      configurationMessage.setEnableJointVelocityLimits(true);
      ikCommandInputManager.submitMessage(configurationMessage);

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
         defaultPelvisMessage.getDesiredPositionInWorld().set(lockPelvisPose.getPosition());
         defaultPelvisMessage.getDesiredOrientationInWorld().set(lockPelvisPose.getOrientation());
         defaultPelvisMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         defaultPelvisMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         MessageTools.packWeightMatrix3DMessage(defaultPelvisMessageLockWeight.getValue(), defaultPelvisMessage.getLinearWeightMatrix());
         MessageTools.packWeightMatrix3DMessage(defaultPelvisMessageLockWeight.getValue(), defaultPelvisMessage.getAngularWeightMatrix());
      }
      else
      {
         lockPelvisPose.setFromReferenceFrame(pelvis.getBodyFixedFrame());
         defaultPelvisMessage.getDesiredPositionInWorld().set(lockPelvisPose.getPosition());
         defaultPelvisMessage.getDesiredOrientationInWorld().setToYawOrientation(lockPelvisPose.getYaw());
         defaultPelvisMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, true, worldFrame));
         defaultPelvisMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         MessageTools.packWeightMatrix3DMessage(defaultPelvisMessageLinearWeight.getValue(), defaultPelvisMessage.getLinearWeightMatrix());
         MessageTools.packWeightMatrix3DMessage(defaultPelvisMessageAngularWeight.getValue(), defaultPelvisMessage.getAngularWeightMatrix());
      }

      lockChest.set(tools.getConfigurationCommand().isLockChest());

      if (lockChest.getValue())
      {
         lockChestPose.setFromReferenceFrame(controllerFullRobotModel.getChest().getBodyFixedFrame());
         defaultChestMessage.getDesiredPositionInWorld().set(lockChestPose.getPosition());
         defaultChestMessage.getDesiredOrientationInWorld().set(lockChestPose.getOrientation());
         defaultChestMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         defaultChestMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         MessageTools.packWeightMatrix3DMessage(defaultChestMessageLockWeight.getValue(), defaultChestMessage.getLinearWeightMatrix());
         MessageTools.packWeightMatrix3DMessage(defaultChestMessageLockWeight.getValue(), defaultChestMessage.getAngularWeightMatrix());
      }
      else
      {
         lockChestPose.setFromReferenceFrame(chest.getBodyFixedFrame());
         defaultChestMessage.getDesiredPositionInWorld().setToZero();
         defaultChestMessage.getDesiredOrientationInWorld().setToYawOrientation(lockChestPose.getYaw());
         defaultChestMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(false, false, false, worldFrame));
         defaultChestMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(true, true, true, worldFrame));
         MessageTools.packWeightMatrix3DMessage(0.0, defaultChestMessage.getLinearWeightMatrix());
         MessageTools.packWeightMatrix3DMessage(defaultChestMessageAngularWeight.getValue(), defaultChestMessage.getAngularWeightMatrix());
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

      //      TObjectDoubleHashMap<OneDoFJointBasics> initialRobotConfigurationMap = ikController.getInitialRobotConfigurationMap();
      //
      //      if (initialRobotConfigurationMap != null)
      //      {
      //         for (RobotSide robotSide : RobotSide.values)
      //         {
      //            OneDoFJointBasics[] joints = armJoints.get(robotSide);
      //            List<KinematicsToolboxOneDoFJointMessage> preferredMessages = preferredArmJointMessages.get(robotSide);
      //
      //            for (int i = 0; i < joints.length; i++)
      //            {
      //               OneDoFJointBasics joint = joints[i];
      //               preferredMessages.get(i).setDesiredPosition(initialRobotConfigurationMap.get(joint.getName()));
      //            }
      //         }
      //      }
      resetFilter = true;
      streamingStartTime.set(Double.NaN);

      ikRobotState.setToNaN();
      initialRobotState.setToNaN();
      blendedRobotState.setToNaN();
      filteredRobotState.setToNaN();
      outputRobotState.setToNaN();

      timeOfLastInput.set(Double.NaN);
      timeSinceLastInput.set(Double.NaN);
      inputFrequency.reset();

      for (YoFramePoint3D rawInputPosition : rawInputPositionMap.values())
         rawInputPosition.setToNaN();

      for (YoFramePoint3D rawExtrapolatedInputPosition : rawExtrapolatedInputPositionMap.values())
         rawExtrapolatedInputPosition.setToNaN();

      for (AlphaFilteredYoFramePoint filteredExtrapolatedInputPosition : filteredExtrapolatedInputPositionMap.values())
      {
         filteredExtrapolatedInputPosition.setToNaN();
         filteredExtrapolatedInputPosition.reset();
      }

      for (YoFrameQuaternion rawInputOrientation : rawInputOrientationMap.values())
         rawInputOrientation.setToNaN();

      for (YoFrameQuaternion rawExtrapolatedInputOrientation : rawExtrapolatedInputOrientationMap.values())
         rawExtrapolatedInputOrientation.setToNaN();

      for (AlphaFilteredYoFrameQuaternion filteredExtrapolatedInputOrientation : filteredExtrapolatedInputOrientationMap.values())
      {
         filteredExtrapolatedInputOrientation.setToNaN();
         filteredExtrapolatedInputOrientation.reset();
      }
      resetEstimatedInputVelocities();

      System.gc();
   }

   private void resetEstimatedInputVelocities()
   {
      for (YoFixedFrameSpatialVector rawInputSpatialVelocity : rawInputSpatialVelocityArray)
         rawInputSpatialVelocity.setToZero();

      for (YoFixedFrameSpatialVector decayingInputSpatialVelocity : decayingInputSpatialVelocityArray)
         decayingInputSpatialVelocity.setToZero();
   }

   private final KinematicsStreamingToolboxInputCommand rawInputs = new KinematicsStreamingToolboxInputCommand();
   private final KinematicsStreamingToolboxInputCommand filteredInputs = new KinematicsStreamingToolboxInputCommand();

   @Override
   public void doAction(double timeInState)
   {
      tools.pollInputCommand();

      FullHumanoidRobotModel controllerFullRobotModel = tools.getCurrentFullRobotModel();

      if (lockPelvis.getValue() && !tools.getConfigurationCommand().isPelvisTaskspaceEnabled())
      {
         lockPelvisPose.setFromReferenceFrame(controllerFullRobotModel.getPelvis().getBodyFixedFrame());
         defaultPelvisMessage.getDesiredPositionInWorld().set(lockPelvisPose.getPosition());
         defaultPelvisMessage.getDesiredOrientationInWorld().set(lockPelvisPose.getOrientation());
      }

      if (lockChest.getValue() && !tools.getConfigurationCommand().isChestTaskspaceEnabled())
      {
         lockChestPose.setFromReferenceFrame(controllerFullRobotModel.getChest().getBodyFixedFrame());
         defaultChestMessage.getDesiredPositionInWorld().set(lockChestPose.getPosition());
         defaultChestMessage.getDesiredOrientationInWorld().set(lockChestPose.getOrientation());
      }

      estimateInputsVelocity();

      KinematicsStreamingToolboxInputCommand latestInput = tools.getLatestInput();

      if (latestInput != null)
      {
         for (int i = 0; i < latestInput.getNumberOfInputs(); i++)
         { // Sets the user weights only if provided.
            setDefaultWeightIfNeeded(latestInput.getInput(i).getSelectionMatrix(), latestInput.getInput(i).getWeightMatrix());
         }

         if (!latestInput.getStreamToController() && latestInput.getNumberOfInputs() == 0 && tools.hasPreviousInput())
         {
            /*
             * In case the abruptly stops streaming and the message is empty. Only then we remember the last
             * inputs and use them to finish this session. Without this, the robot would go to its privileged
             * configuration.
             */
            latestInput.addInputs(tools.getPreviousInput().getInputs());
            filteredInputs.set(latestInput);
         }

         if (tools.hasNewInputCommand())
         {
            for (int i = 0; i < latestInput.getNumberOfInputs(); i++)
            { // This is just for viz purpose
               KinematicsToolboxRigidBodyCommand input = latestInput.getInput(i);
               RigidBodyBasics endEffector = input.getEndEffector();
               FramePose3D desiredPose = input.getDesiredPose();

               YoFramePoint3D rawInputPosition = rawInputPositionMap.get(endEffector);
               if (rawInputPosition == null)
                  continue;
               YoFrameQuaternion rawInputOrientation = rawInputOrientationMap.get(endEffector);

               rawInputPosition.set(desiredPose.getPosition());
               rawInputOrientation.set(desiredPose.getOrientation());
            }

            rawInputs.set(latestInput);
         }
         else
         {
            extrapolateInputPositions(rawInputs, toolboxControllerPeriod);
         }

         filteredInputs.set(rawInputs);

         for (int i = 0; i < filteredInputs.getNumberOfInputs(); i++)
         {
            /*
             * Extract info from user inputs and filter position and orientation of each controlled body.
             */
            KinematicsToolboxRigidBodyCommand filteredInput = filteredInputs.getInput(i);
            RigidBodyBasics endEffector = filteredInput.getEndEffector();
            FramePose3D desiredPose = filteredInput.getDesiredPose();

            if (lockPelvis.getValue() && endEffector == pelvis)
               continue;
            if (lockChest.getValue() && endEffector == chest)
               continue;

            YoFramePoint3D rawInputPosition = rawExtrapolatedInputPositionMap.get(endEffector);

            if (rawInputPosition == null)
               continue;

            AlphaFilteredYoFramePoint filteredInputPosition = filteredExtrapolatedInputPositionMap.get(endEffector);

            YoFrameQuaternion rawInputOrientation = rawExtrapolatedInputOrientationMap.get(endEffector);
            AlphaFilteredYoFrameQuaternion filteredInputOrientation = filteredExtrapolatedInputOrientationMap.get(endEffector);

            rawInputPosition.set(desiredPose.getPosition());
            rawInputOrientation.set(desiredPose.getOrientation());

            // Update filters
            filteredInputPosition.update();
            filteredInputOrientation.update();

            desiredPose.getPosition().set(filteredInputPosition);
            desiredPose.getOrientation().set(filteredInputOrientation);

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
            //            else
            //            {
            //               List<KinematicsToolboxOneDoFJointMessage> preferredMessages = preferredArmJointMessages.get(robotSide);
            //               for (int i = 0; i < preferredMessages.size(); i++)
            //               {
            //                  KinematicsToolboxOneDoFJointMessage preferredMessage = preferredMessages.get(i);
            //                  preferredMessage.setWeight(preferredArmConfigWeight.getValue());
            //                  ikCommandInputManager.submitMessage(preferredMessage);
            //               }
            //            }
         }

         isStreaming.set(latestInput.getStreamToController());
         if (latestInput.getStreamInitialBlendDuration() > 0.0)
            streamingBlendingDuration.set(latestInput.getStreamInitialBlendDuration());
         else
            streamingBlendingDuration.set(defautlInitialBlendDuration);
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
      ikRobotState.set(tools.getIKController().getSolution());

      /*
       * Updating the desired default position for the arms and/or neck only if the corresponding
       * end-effector is controlled. This allows to improve continuity in case the user stops controlling
       * a hand/head.
       */
      if (latestInput != null)
      {
         if (latestInput.hasInputFor(head))
         {
            for (int i = 0; i < neckJoints.length; i++)
               defaultNeckJointMessages.get(i).setDesiredPosition(neckJoints[i].getQ());
         }

         for (RobotSide robotSide : RobotSide.values)
         {
            if (latestInput.hasInputFor(hands.get(robotSide)))
            {
               OneDoFJointBasics[] joints = armJoints.get(robotSide);
               List<KinematicsToolboxOneDoFJointMessage> messages = defaultArmJointMessages.get(robotSide);

               for (int i = 0; i < joints.length; i++)
                  messages.get(i).setDesiredPosition(joints[i].getQ());
            }
         }
      }

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

         double timeSinceLastPublished = timeInState - timeOfLastMessageSentToController.getValue();

         if (timeSinceLastPublished >= publishingPeriod.getValue())
         {
            outputRobotState.set(filteredRobotState);
            outputRobotState.scaleVelocities(outputJointVelocityScale.getValue());

            if (timeInBlending < streamingBlendingDuration.getValue())
            {
               double alpha = MathTools.clamp(timeInBlending / streamingBlendingDuration.getValue(), 0.0, 1.0);
               double alphaDot = 1.0 / streamingBlendingDuration.getValue();
               blendedRobotState.interpolate(initialRobotState.getStatus(), outputRobotState.getStatus(), alpha, alphaDot);
               if (streamingMessagePublisher == null || !useStreamingPublisher.getValue())
                  trajectoryMessagePublisher.publish(tools.setupTrajectoryMessage(blendedRobotState.getStatus()));
               else
                  streamingMessagePublisher.publish(tools.setupStreamingMessage(blendedRobotState.getStatus()));
            }
            else
            {
               if (streamingMessagePublisher == null || !useStreamingPublisher.getValue())
                  trajectoryMessagePublisher.publish(tools.setupTrajectoryMessage(outputRobotState.getStatus()));
               else
                  streamingMessagePublisher.publish(tools.setupStreamingMessage(outputRobotState.getStatus()));
            }

            timeOfLastMessageSentToController.set(timeInState);
         }
      }
      else
      {
         if (wasStreaming.getValue())
         {
            outputRobotState.set(filteredRobotState);
            outputRobotState.scaleVelocities(outputJointVelocityScale.getValue());
            trajectoryMessagePublisher.publish(tools.setupFinalizeTrajectoryMessage(outputRobotState.getStatus()));
         }

         timeOfLastMessageSentToController.set(Double.NEGATIVE_INFINITY);
      }

      wasStreaming.set(isStreaming.getValue());
   }

   private void estimateInputsVelocity()
   {
      if (!tools.hasPreviousInput())
      {
         resetEstimatedInputVelocities();
         return;
      }

      if (!tools.hasNewInputCommand())
      {
         decayEstimatedInputVelocity();
         return;
      }

      // The inputs just got updated, need to recompute velocities.
      KinematicsStreamingToolboxInputCommand latestInput = tools.getLatestInput();
      KinematicsStreamingToolboxInputCommand previousInput = tools.getPreviousInput();

      if (latestInput.getNumberOfInputs() != previousInput.getNumberOfInputs())
      {
         resetEstimatedInputVelocities();
         return;
      }

      double latestInputReceivedTime = Conversions.nanosecondsToSeconds(latestInput.getTimestamp());
      double previousInputReceivedTime = Conversions.nanosecondsToSeconds(previousInput.getTimestamp());

      double timeInterval = latestInputReceivedTime - previousInputReceivedTime;

      if (timeInterval <= 0.0)
      {
         LogTools.warn("Got a negative or zero time interval between 2 inputs: " + timeInterval);
         decayEstimatedInputVelocity();
         return;
      }

      for (int i = 0; i < latestInput.getNumberOfInputs(); i++)
      {
         RigidBodyBasics endEffector = latestInput.getInput(i).getEndEffector();
         YoFixedFrameSpatialVector rawSpatialVelocity = rawInputSpatialVelocityMap.get(endEffector);

         if (rawSpatialVelocity == null)
            continue;

         /*
          * We compute the velocity of the inputs and then do a 1st-order extrapolation in the future and
          * update the raw input. This way, if for the next control tick we didn't get any new inputs, the IK
          * keep moving which in result should improve continuity of any motion.
          */
         FramePose3D previousInputPose = previousInput.getInput(i).getDesiredPose();
         FixedFramePoint3DBasics previousInputPosition = previousInputPose.getPosition();
         FixedFrameQuaternionBasics previousInputOrientation = previousInputPose.getOrientation();

         FramePose3D latestInputPose = latestInput.getInput(i).getDesiredPose();
         FixedFramePoint3DBasics latestInputPosition = latestInputPose.getPosition();
         FixedFrameQuaternionBasics latestInputOrientation = latestInputPose.getOrientation();

         YoFrameVector3D rawLinearVelocity = rawSpatialVelocity.getLinearPart();
         YoFrameVector3D rawAngularVelocity = rawSpatialVelocity.getAngularPart();

         KSTTools.computeLinearVelocity(timeInterval, previousInputPosition, latestInputPosition, rawLinearVelocity);
         KSTTools.computeAngularVelocity(timeInterval, previousInputOrientation, latestInputOrientation, rawAngularVelocity);

         decayingInputSpatialVelocityMap.get(endEffector).set(rawSpatialVelocity);
         inputVelocityDecayFactor.set(0.0);
      }
   }

   private void decayEstimatedInputVelocity()
   {
      double alpha = Math.min(1.0, inputVelocityDecayFactor.getValue() + toolboxControllerPeriod / inputVelocityDecayDuration.getValue());
      inputVelocityDecayFactor.set(alpha);

      for (int i = 0; i < decayingInputSpatialVelocityArray.length; i++)
      {
         YoFixedFrameSpatialVector rawVelocity = rawInputSpatialVelocityArray[i];
         YoFixedFrameSpatialVector decayingVelocity = decayingInputSpatialVelocityArray[i];
         decayingVelocity.getLinearPart().interpolate(rawVelocity.getLinearPart(), EuclidCoreTools.zeroVector3D, alpha);
         decayingVelocity.getAngularPart().interpolate(rawVelocity.getAngularPart(), EuclidCoreTools.zeroVector3D, alpha);
      }
   }

   private void extrapolateInputPositions(KinematicsStreamingToolboxInputCommand inputs, double integrationDT)
   {
      for (int i = 0; i < inputs.getNumberOfInputs(); i++)
      {
         KinematicsToolboxRigidBodyCommand input = inputs.getInput(i);
         YoFixedFrameSpatialVector inputVelocity = decayingInputSpatialVelocityMap.get(input.getEndEffector());

         FramePose3D desiredPose = input.getDesiredPose();
         KSTTools.integrateLinearVelocity(integrationDT, desiredPose.getPosition(), inputVelocity.getLinearPart(), desiredPose.getPosition());
         KSTTools.integrateAngularVelocity(integrationDT, desiredPose.getOrientation(), inputVelocity.getLinearPart(), desiredPose.getOrientation());
      }
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
         decayingInput.getWeightMatrix().scale(inputDecayFactor.getValue());

         if (findMaximumWeightValue(decayingInput.getWeightMatrix()) < 0.1)
            decayingInputs.remove(i);
      }

      numberOfDecayingInputs.set(decayingInputs.size());
   }

   private double findMaximumWeightValue(WeightMatrix6D weightMatrix)
   {
      return Math.max(findMaximumWeightValue(weightMatrix.getAngularPart()), findMaximumWeightValue(weightMatrix.getLinearPart()));
   }

   private double findMaximumWeightValue(WeightMatrix3D weightMatrix)
   {
      return EuclidCoreTools.max(weightMatrix.getX(), weightMatrix.getY(), weightMatrix.getZ());
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
      tools.flushInputCommands();
   }
}
