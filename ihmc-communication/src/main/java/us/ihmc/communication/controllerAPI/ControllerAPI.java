package us.ihmc.communication.controllerAPI;

import controller_msgs.*;
import ihmc_common_msgs.MessageCollection;
import ihmc_common_msgs.MessageCollectionNotification;
import ihmc_common_msgs.Point2DMessage;
import ihmc_common_msgs.TextToSpeechPacket;
import toolbox_msgs.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Topic;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Base API for the IHMC control API.
 * <p>
 * For the humanoid controller, see {@link us.ihmc.communication.HumanoidControllerAPI}.
 */
public final class ControllerAPI
{
   public static final Set<Class<?>> inputMessageClasses = new HashSet<>();
   public static final Set<Class<? extends ROS2Message<?>>> outputMessageClasses = new HashSet<>();

   public static final Map<Class<?>, ROS2QoSProfile> inputMessageClassSpecificQoS = new HashMap<>();
   public static final Map<Class<?>, ROS2QoSProfile> outputMessageClassSpecificQoS = new HashMap<>();

   static
   {
      // Commands supported by bipedal walking controller WalkingControllerState
      inputMessageClasses.add(ArmTrajectoryMessage.class);
      inputMessageClasses.add(HandTrajectoryMessage.class);
      inputMessageClasses.add(LegTrajectoryMessage.class);
      inputMessageClasses.add(FootTrajectoryMessage.class);
      inputMessageClasses.add(HeadTrajectoryMessage.class);
      inputMessageClasses.add(NeckTrajectoryMessage.class);
      inputMessageClasses.add(NeckDesiredAccelerationsMessage.class);
      inputMessageClasses.add(ChestTrajectoryMessage.class);
      inputMessageClasses.add(SpineTrajectoryMessage.class);
      inputMessageClasses.add(PelvisTrajectoryMessage.class);
      inputMessageClasses.add(PelvisOrientationTrajectoryMessage.class);
      inputMessageClasses.add(PelvisHeightTrajectoryMessage.class);
      inputMessageClasses.add(StopAllTrajectoryMessage.class);
      inputMessageClasses.add(FootstepDataListMessage.class);
      inputMessageClasses.add(GoHomeMessage.class);
      inputMessageClasses.add(FootLoadBearingMessage.class);
      inputMessageClasses.add(ArmDesiredAccelerationsMessage.class);
      inputMessageClasses.add(AutomaticManipulationAbortMessage.class);
      inputMessageClasses.add(HighLevelStateMessage.class);
      inputMessageClasses.add(AbortWalkingMessage.class);
      inputMessageClasses.add(PrepareForLocomotionMessage.class);
      inputMessageClasses.add(PauseWalkingMessage.class);
      inputMessageClasses.add(ReinitializeStateEstimatorMessage.class);
      inputMessageClasses.add(SpineDesiredAccelerationsMessage.class);
      inputMessageClasses.add(HandLoadBearingMessage.class);
      inputMessageClasses.add(HandHybridJointspaceTaskspaceTrajectoryMessage.class);
      inputMessageClasses.add(HeadHybridJointspaceTaskspaceTrajectoryMessage.class);
      inputMessageClasses.add(ChestHybridJointspaceTaskspaceTrajectoryMessage.class);
      inputMessageClasses.add(ClearDelayQueueMessage.class);
      inputMessageClasses.add(MomentumTrajectoryMessage.class);
      inputMessageClasses.add(CenterOfMassTrajectoryMessage.class);
      inputMessageClasses.add(HandWrenchTrajectoryMessage.class);

      // Commands supported by the fast-walking controller, not in this repo
      inputMessageClasses.add(VelocityBasedWalkingInputMessage.class);
      inputMessageClasses.add(FastWalkingGaitParametersMessage.class);

      // Commands supported by multi-contact controller, not in this repo
      inputMessageClasses.add(MultiContactTrajectoryMessage.class);
      inputMessageClasses.add(MultiContactTrajectorySequenceMessage.class);
      inputMessageClasses.add(MultiContactBalanceStatus.class);
      inputMessageClasses.add(MultiContactTimedContactSequenceMessage.class);

      // Command supported by the joint-space controller JointspacePositionControllerState
      inputMessageClasses.add(WholeBodyJointspaceTrajectoryMessage.class);

      // Commands supported by the RL controller, not in this repo
      inputMessageClasses.add(Point2DMessage.class);
      inputMessageClasses.add(RLModelSelectionMessage.class);

      // Toolbox management
      inputMessageClasses.add(ToolboxStateMessage.class);

      // Commands supported by the kinematics toolbox
      inputMessageClasses.add(KinematicsToolboxCenterOfMassMessage.class);
      inputMessageClasses.add(KinematicsToolboxRigidBodyMessage.class);
      inputMessageClasses.add(KinematicsToolboxOneDoFJointMessage.class);
      inputMessageClasses.add(KinematicsToolboxConfigurationMessage.class);
      inputMessageClasses.add(KinematicsToolboxSupportRegionMessage.class);
      inputMessageClasses.add(KinematicsToolboxPrivilegedConfigurationMessage.class);
      inputMessageClasses.add(KinematicsToolboxInputCollectionMessage.class);
      inputMessageClasses.add(HumanoidKinematicsToolboxConfigurationMessage.class);

      // Commands supported by the kinematics streaming toolbox
      inputMessageClasses.add(KinematicsStreamingToolboxInputMessage.class);
      inputMessageClasses.add(KinematicsStreamingToolboxConfigurationMessage.class);

      // Input messages that don't have a corresponding command
      inputMessageClasses.add(MessageCollection.class);
      inputMessageClasses.add(WholeBodyTrajectoryMessage.class);
      inputMessageClasses.add(WholeBodyStreamingMessage.class);

      // Robot startup messages
      inputMessageClasses.add(EStopMasterGainCommandMessage.class);

      // Statuses supported by bipedal walking controller {@link WalkingControllerState}
      outputMessageClasses.add(CapturabilityBasedStatus.class);
      outputMessageClasses.add(FootstepStatusMessage.class);
      outputMessageClasses.add(PlanOffsetStatus.class);
      outputMessageClasses.add(WalkingStatusMessage.class);
      outputMessageClasses.add(WalkingControllerFailureStatusMessage.class);
      outputMessageClasses.add(ManipulationAbortedStatus.class);
      outputMessageClasses.add(HighLevelStateChangeStatusMessage.class);
      outputMessageClasses.add(TextToSpeechPacket.class);
      outputMessageClasses.add(ControllerCrashNotificationPacket.class);
      outputMessageClasses.add(JointspaceTrajectoryStatusMessage.class);
      outputMessageClasses.add(TaskspaceTrajectoryStatusMessage.class);
      outputMessageClasses.add(JointDesiredOutputMessage.class);
      outputMessageClasses.add(RobotDesiredConfigurationData.class);
      outputMessageClasses.add(FootstepQueueStatusMessage.class);
      outputMessageClasses.add(QueuedFootstepStatusMessage.class);
      outputMessageClasses.add(WrenchTrajectoryStatusMessage.class);
      outputMessageClasses.add(InvalidPacketNotificationPacket.class);
      outputMessageClasses.add(MessageCollectionNotification.class);

      // Statuses supported by the kinematics toolbox
      outputMessageClasses.add(KinematicsToolboxOutputStatus.class);

      // Statuses supported by multi-contact controller, not in this repo
      outputMessageClasses.add(MultiContactBalanceStatus.class);
      outputMessageClasses.add(MultiContactTrajectoryStatus.class);

      // Robot hardware status messages
      outputMessageClasses.add(EStopMasterGainStatusMessage.class);

      // RL policy state (available models and current selection)
      outputMessageClasses.add(RLPolicyState.class);

      // Setting the input messages with specific QoS
      inputMessageClassSpecificQoS.put(WholeBodyStreamingMessage.class, ROS2QoSProfile.BEST_EFFORT);
      inputMessageClassSpecificQoS.put(KinematicsStreamingToolboxInputMessage.class, ROS2QoSProfile.BEST_EFFORT);

      // Setting the output messages with specific QoS
      outputMessageClassSpecificQoS.put(CapturabilityBasedStatus.class, ROS2QoSProfile.BEST_EFFORT);
      outputMessageClassSpecificQoS.put(JointDesiredOutputMessage.class, ROS2QoSProfile.BEST_EFFORT);
      outputMessageClassSpecificQoS.put(RobotDesiredConfigurationData.class, ROS2QoSProfile.BEST_EFFORT);
      outputMessageClassSpecificQoS.put(FootstepQueueStatusMessage.class, ROS2QoSProfile.BEST_EFFORT);
      outputMessageClassSpecificQoS.put(MultiContactBalanceStatus.class, ROS2QoSProfile.BEST_EFFORT);
   }

   public static ROS2Topic<?> getBaseTopic(String controlModuleName, String robotName)
   {
      return ROS2Tools.IHMC_ROOT.appendedWith(controlModuleName).appendedWith(robotName);
   }

   @SuppressWarnings("unchecked")
   public static <T extends ROS2Message<T>> ROS2Topic<T> getTopic(ROS2Topic<?> baseTopic, Class<T> messageClass)
   {
      if (inputMessageClasses.contains(messageClass))
         return (ROS2Topic<T>) baseTopic.appendedWith("input").withType(messageClass);
      else if (outputMessageClasses.contains(messageClass))
         return (ROS2Topic<T>) baseTopic.appendedWith("output").withType(messageClass);
      else
         return (ROS2Topic<T>) baseTopic.withType(messageClass);
   }

   public static <T extends ROS2Message<T>> ROS2Topic<T> getLowFrequencyTopic(ROS2Topic<?> baseTopic, Class<T> messageClass)
   {
      return getTopic(baseTopic, messageClass).appendedWith("lf");
   }

   public static ROS2QoSProfile getQoS(Class<?> messageClass)
   {
      if (inputMessageClasses.contains(messageClass))
         return Objects.requireNonNullElse(inputMessageClassSpecificQoS.get(messageClass), ROS2QoSProfile.RELIABLE);
      else if (outputMessageClasses.contains(messageClass))
         return Objects.requireNonNullElse(outputMessageClassSpecificQoS.get(messageClass), ROS2QoSProfile.RELIABLE);
      else
         return ROS2QoSProfile.BEST_EFFORT;
   }
}
