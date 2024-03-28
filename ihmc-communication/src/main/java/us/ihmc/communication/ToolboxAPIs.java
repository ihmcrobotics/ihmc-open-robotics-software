package us.ihmc.communication;

import toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage;
import toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import us.ihmc.ros2.ROS2Topic;

public class ToolboxAPIs
{
   public static final String CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME = "toolbox/continuous_planning";
   public static final String FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME = "toolbox/footstep_postprocessing";
   public static final String KINEMATICS_TOOLBOX_MODULE_NAME = "toolbox/ik";
   public static final String KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME = "toolbox/ik_planning";
   public static final String KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME = "toolbox/ik_streaming";
   public static final String STEP_CONSTRAINT_TOOLBOX_MODULE_NAME = "/toolbox/step_constraint";
   public static final String WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME = "toolbox/ik_trajectory";
   public static final String WALKING_PREVIEW_TOOLBOX_MODULE_NAME = "toolbox/walking_controller_preview";
   public static final String EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME = "toolbox/external_force_estimation";
   public static final String STEP_TELEOP_TOOLBOX_MODULE_NAME = "toolbox/teleop/step_teleop";
   public static final String DIRECTIONAL_CONTROL_TOOLBOX_MODULE_NAME = "/toolbox/directional_control";

   public static final ROS2Topic<?> CONTINUOUS_PLANNING_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(CONTINUOUS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> FOOTSTEP_POSTPROCESSING_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(FOOTSTEP_POSTPROCESSING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> KINEMATICS_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(KINEMATICS_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> KINEMATICS_PLANNING_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(KINEMATICS_PLANNING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> KINEMATICS_STREAMING_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(KINEMATICS_STREAMING_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> STEP_CONSTRAINT_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(STEP_CONSTRAINT_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> WHOLE_BODY_TRAJECTORY_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> WALKING_PREVIEW_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(WALKING_PREVIEW_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> EXTERNAL_FORCE_ESTIMATION_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(EXTERNAL_FORCE_ESTIMATION_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> STEP_TELEOP_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(STEP_TELEOP_TOOLBOX_MODULE_NAME);
   public static final ROS2Topic<?> DIRECTIONAL_CONTROL_TOOLBOX = ROS2Tools.IHMC_ROOT.withModule(DIRECTIONAL_CONTROL_TOOLBOX_MODULE_NAME);

   public static ROS2Topic<WalkingControllerPreviewInputMessage> getControllerPreviewInputTopic(String robotName)
   {
      return WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withInput().withTypeName(WalkingControllerPreviewInputMessage.class);
   }

   public static ROS2Topic<WalkingControllerPreviewOutputMessage> getControllerPreviewOutputTopic(String robotName)
   {
      return WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withOutput().withTypeName(WalkingControllerPreviewOutputMessage.class);
   }
}
