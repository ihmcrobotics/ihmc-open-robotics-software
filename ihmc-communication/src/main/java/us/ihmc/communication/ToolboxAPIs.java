package us.ihmc.communication;

import toolbox_msgs.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.ToolboxStateMessage;
import toolbox_msgs.WalkingControllerPreviewInputMessage;
import toolbox_msgs.WalkingControllerPreviewOutputMessage;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Topic;

public final class ToolboxAPIs
{
   public static final String FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME = "toolbox/footstep_postprocessing";
   public static final String KINEMATICS_TOOLBOX_MODULE_NAME = "toolbox/ik";
   public static final String KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME = "toolbox/ik_planning";
   public static final String KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME = "toolbox/ik_streaming";
   public static final String FOOTSTEP_STREAMING_TOOLBOX_MODULE_NAME = "toolbox/footstep_streaming";
   public static final String STEP_CONSTRAINT_TOOLBOX_MODULE_NAME = "/toolbox/step_constraint";
   public static final String WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME = "toolbox/ik_trajectory";
   public static final String WALKING_PREVIEW_TOOLBOX_MODULE_NAME = "toolbox/walking_controller_preview";
   public static final String EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME = "toolbox/external_force_estimation";
   public static final String STEP_TELEOP_TOOLBOX_MODULE_NAME = "toolbox/teleop/step_teleop";
   public static final String DIRECTIONAL_CONTROL_TOOLBOX_MODULE_NAME = "/toolbox/directional_control";

   public static final HumanoidROS2Topic<?> FOOTSTEP_POSTPROCESSING_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> KINEMATICS_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(KINEMATICS_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> KINEMATICS_PLANNING_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> KINEMATICS_STREAMING_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> FOOTSTEP_STREAMING_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(FOOTSTEP_STREAMING_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> STEP_CONSTRAINT_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(STEP_CONSTRAINT_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> WHOLE_BODY_TRAJECTORY_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> WALKING_PREVIEW_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(WALKING_PREVIEW_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> EXTERNAL_FORCE_ESTIMATION_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> STEP_TELEOP_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(STEP_TELEOP_TOOLBOX_MODULE_NAME);
   public static final HumanoidROS2Topic<?> DIRECTIONAL_CONTROL_TOOLBOX = HumanoidROS2Topic.IHMC_ROOT.withModule(DIRECTIONAL_CONTROL_TOOLBOX_MODULE_NAME);

   public static ROS2Topic<WalkingControllerPreviewInputMessage> getControllerPreviewInputTopic(String robotName)
   {
      return WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withInput().withTypeName(WalkingControllerPreviewInputMessage.class);
   }

   public static ROS2Topic<WalkingControllerPreviewOutputMessage> getControllerPreviewOutputTopic(String robotName)
   {
      return WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withOutput().withTypeName(WalkingControllerPreviewOutputMessage.class);
   }

   public static <T extends ROS2Message<T>> ROS2Topic<T> getIKToolboxTopic(Class<T> messageClass, String robotName)
   {
      return ControllerAPI.getTopic(ControllerAPI.getBaseTopic(KINEMATICS_TOOLBOX_MODULE_NAME, robotName), messageClass);
   }

   public static ROS2Topic<?> getIKStreamingInputBaseTopic(String robotName)
   {
      return KINEMATICS_STREAMING_TOOLBOX.withRobot(robotName).withInput();
   }

   public static ROS2Topic<KinematicsStreamingToolboxInputMessage> getIKStreamingInputTopic(String robotName)
   {
      return ControllerAPI.getTopic(getIKStreamingInputBaseTopic(robotName), KinematicsStreamingToolboxInputMessage.class);
   }

   public static ROS2Topic<ToolboxStateMessage> getIKStreamingStateTopic(String robotName)
   {
      return ControllerAPI.getTopic(getIKStreamingInputBaseTopic(robotName), ToolboxStateMessage.class);
   }

   public static ROS2Topic<KinematicsToolboxConfigurationMessage> getInputToolboxConfigurationTopic(String robotName)
   {
      return ControllerAPI.getTopic(getIKStreamingInputBaseTopic(robotName), KinematicsToolboxConfigurationMessage.class);
   }
}
