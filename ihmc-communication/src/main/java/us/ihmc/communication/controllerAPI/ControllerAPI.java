package us.ihmc.communication.controllerAPI;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.MessageCollection;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import java.util.HashSet;

/**
 * Base API for the IHMC control API.
 *
 * For the humanoid controller, see {@link us.ihmc.communication.HumanoidControllerAPI}.
 */
public class ControllerAPI
{
   public static final HashSet<Class<?>> inputMessageClasses = new HashSet<>();
   public static final HashSet<Class<?>> outputMessageClasses = new HashSet<>();

   static
   {
      /** Commands supported by bipedal walking controller {@link WalkingControllerState} */
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

      /** Commands supported by the fast-walking controller, not in this repo */
      inputMessageClasses.add(DirectionalControlInputMessage.class);
      inputMessageClasses.add(FastWalkingGaitParametersMessage.class);

      /** Commands supported by multi-contact controller, not in this repo */
      inputMessageClasses.add(MultiContactTrajectoryMessage.class);
      inputMessageClasses.add(MultiContactTrajectorySequenceMessage.class);
      inputMessageClasses.add(MultiContactBalanceStatus.class);
      inputMessageClasses.add(MultiContactTimedContactSequenceMessage.class);

      /** Commands supported by the Crocoddyl control state */
      inputMessageClasses.add(CrocoddylSolverTrajectoryMessage.class);

      /** Command supported by the joint-space controller {@link JointspacePositionControllerState} */
      inputMessageClasses.add(WholeBodyJointspaceTrajectoryMessage.class);

      // Input messages that don't have a corresponding command
      inputMessageClasses.add(MessageCollection.class);
      inputMessageClasses.add(WholeBodyTrajectoryMessage.class);
      inputMessageClasses.add(WholeBodyStreamingMessage.class);

      /** Statuses supported by bipedal walking controller {@link WalkingControllerState} */
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

      /** Statuses supported by multi-contact controller, not in this repo */
      outputMessageClasses.add(MultiContactBalanceStatus.class);
      outputMessageClasses.add(MultiContactTrajectoryStatus.class);
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
      {
         // Streaming commands should be best effort
         if (messageClass.equals(WholeBodyStreamingMessage.class))
            return ROS2QosProfile.BEST_EFFORT();

         return ROS2QosProfile.RELIABLE();
      }
      else if (outputMessageClasses.contains(messageClass))
      {
         // Periodic topics are should be best effort
         if (messageClass.equals(CapturabilityBasedStatus.class)
          || messageClass.equals(JointDesiredOutputMessage.class)
          || messageClass.equals(RobotDesiredConfigurationData.class)
          || messageClass.equals(FootstepQueueStatusMessage.class)
          || messageClass.equals(MultiContactBalanceStatus.class))
            return ROS2QosProfile.BEST_EFFORT();

         return ROS2QosProfile.RELIABLE();
      }
      else
      {
         return ROS2QosProfile.DEFAULT();
      }
   }
}
