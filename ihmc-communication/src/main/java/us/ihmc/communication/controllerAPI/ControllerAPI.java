package us.ihmc.communication.controllerAPI;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.MessageCollection;
import ihmc_common_msgs.msg.dds.MessageCollectionNotification;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import toolbox_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import java.util.*;

/**
 * Base API for the IHMC control API.
 * <p>
 * For the humanoid controller, see {@link us.ihmc.communication.HumanoidControllerAPI}.
 */
public final class ControllerAPI
{
   public static final Set<Class<?>> inputMessageClasses = new HashSet<>();
   public static final Set<Class<? extends Settable<?>>> outputMessageClasses = new HashSet<>();

   public static final Map<Class<?>, ROS2QosProfile> inputMessageClassSpecificQoS = new HashMap<>();
   public static final Map<Class<?>, ROS2QosProfile> outputMessageClassSpecificQoS = new HashMap<>();

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
      inputMessageClasses.add(DirectionalControlInputMessage.class);
      inputMessageClasses.add(FastWalkingGaitParametersMessage.class);

      // Commands supported by multi-contact controller, not in this repo
      inputMessageClasses.add(MultiContactTrajectoryMessage.class);
      inputMessageClasses.add(MultiContactTrajectorySequenceMessage.class);
      inputMessageClasses.add(MultiContactBalanceStatus.class);
      inputMessageClasses.add(MultiContactTimedContactSequenceMessage.class);

      // Commands supported by the Crocoddyl control state
      inputMessageClasses.add(CrocoddylSolverTrajectoryMessage.class);

      // Command supported by the joint-space controller JointspacePositionControllerState
      inputMessageClasses.add(WholeBodyJointspaceTrajectoryMessage.class);

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

      // Setting the input messages with specific QoS
      inputMessageClassSpecificQoS.put(WholeBodyStreamingMessage.class, ROS2QosProfile.BEST_EFFORT());
      inputMessageClassSpecificQoS.put(KinematicsStreamingToolboxInputMessage.class, ROS2QosProfile.BEST_EFFORT());

      // Setting the output messages with specific QoS
      outputMessageClassSpecificQoS.put(CapturabilityBasedStatus.class, ROS2QosProfile.BEST_EFFORT());
      outputMessageClassSpecificQoS.put(JointDesiredOutputMessage.class, ROS2QosProfile.BEST_EFFORT());
      outputMessageClassSpecificQoS.put(RobotDesiredConfigurationData.class, ROS2QosProfile.BEST_EFFORT());
      outputMessageClassSpecificQoS.put(FootstepQueueStatusMessage.class, ROS2QosProfile.BEST_EFFORT());
      outputMessageClassSpecificQoS.put(MultiContactBalanceStatus.class, ROS2QosProfile.BEST_EFFORT());
   }

   public static ROS2Topic<?> getBaseTopic(String controlModuleName, String robotName)
   {
      return ROS2Tools.IHMC_ROOT.withModule(controlModuleName).withRobot(robotName);
   }

   public static <T> ROS2Topic<T> getTopic(ROS2Topic<?> baseTopic, Class<T> messageClass)
   {
      if (inputMessageClasses.contains(messageClass))
      {
         return baseTopic.withInput().withTypeName(messageClass).withQoS(getQoS(messageClass));
      }
      if (outputMessageClasses.contains(messageClass))
      {
         return baseTopic.withOutput().withTypeName(messageClass).withQoS(getQoS(messageClass));
      }

      throw new RuntimeException("Topic does not exist: " + messageClass);
   }

   public static ROS2QosProfile getQoS(Class<?> messageClass)
   {
      if (inputMessageClasses.contains(messageClass))
         return Objects.requireNonNullElse(inputMessageClassSpecificQoS.get(messageClass), ROS2QosProfile.RELIABLE());
      else if (outputMessageClasses.contains(messageClass))
         return Objects.requireNonNullElse(outputMessageClassSpecificQoS.get(messageClass), ROS2QosProfile.RELIABLE());
      else
         return ROS2QosProfile.DEFAULT();
   }
}
