package us.ihmc.communication.controllerAPI;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.MessageCollection;
import ihmc_common_msgs.msg.dds.MessageCollectionNotification;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

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
   /**
    * TODO {@link #getTopic(ROS2Topic, Class)} should be reconfigured to consume a custom QoS provider, or at least this should be configurable for different
    * platforms. As it stands, this implementation applies to every robot that uses this API, which is likely a mistake, particularly since these are messages
    * that are only applicable to the humanoid controllers.
    */
   private static final Map<Class<?>, ROS2QosProfile> messageClassSpecificQoS = new HashMap<>();

   static
   {
      // Setting the input messages with specific QoS
      messageClassSpecificQoS.put(WholeBodyStreamingMessage.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(KinematicsStreamingToolboxInputMessage.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(StampedPosePacket.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(PelvisPoseErrorPacket.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(LocalizationPacket.class, ROS2QosProfile.BEST_EFFORT());

      // Setting the output messages with specific QoS
      messageClassSpecificQoS.put(CapturabilityBasedStatus.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(JointDesiredOutputMessage.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(RobotDesiredConfigurationData.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(FootstepQueueStatusMessage.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(MultiContactBalanceStatus.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(HandJointAnglePacket.class, ROS2QosProfile.BEST_EFFORT());
      messageClassSpecificQoS.put(RobotConfigurationData.class, ROS2QosProfile.BEST_EFFORT());
   }


   public static <T> ROS2Topic<T> getTopic(ROS2Topic<?> baseTopic, Class<T> messageClass)
   {
      return baseTopic.withTypeName(messageClass).withQoS(getQoS(messageClass));
   }

   public static ROS2QosProfile getQoS(Class<?> messageClass)
   {
      if (messageClassSpecificQoS.containsKey(messageClass))
         return Objects.requireNonNullElse(messageClassSpecificQoS.get(messageClass), ROS2QosProfile.RELIABLE());
      else
         return ROS2QosProfile.DEFAULT();
   }
}
