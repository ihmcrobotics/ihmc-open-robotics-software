package us.ihmc.communication;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;
import sensor_msgs.CameraInfo;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.jros2.ROS2Topic;

class HumanoidROS2TopicTest
{
   @Test
   void slotOrderDiffersFromPlainAppend()
   {
      HumanoidROS2Topic<?> moduleBase = HumanoidROS2Topic.IHMC_ROOT.withModule("toolbox/teleop/step_teleop");

      assertEquals("/ihmc/toolbox/teleop/step_teleop", moduleBase.toString());
      assertEquals("/ihmc/atlas/toolbox/teleop/step_teleop/output", moduleBase.withRobot("atlas").withOutput().toString());
      assertNotEquals(moduleBase.appendedWith("atlas").appendedWith("output").toString(),
                      moduleBase.withRobot("atlas").withOutput().toString());
   }

   @Test
   void controllerBaseTopicOrder()
   {
      assertEquals("/ihmc/atlas/controller", ControllerAPI.getBaseTopic("controller", "atlas").toString());
      assertNotEquals("/ihmc/controller/atlas", HumanoidROS2Topic.IHMC_ROOT.appendedWith("controller").appendedWith("atlas").toString());
   }

   @Test
   void isAssignableToJros2Topic()
   {
      ROS2Topic<CameraInfo> topic = HumanoidROS2Topic.IHMC_ROOT.withTypeName(CameraInfo.class);
      assertEquals("/ihmc/camera_info", topic.getName());
   }

   @Test
   void perceptionTopicsMatchDevelopSlotOrder()
   {
      assertEquals("/ihmc/perception", PerceptionAPI.PERCEPTION_MODULE.getName());
      assertEquals("/ihmc/height_map/output/height_map_message", PerceptionAPI.HEIGHT_MAP_MESSAGE.getName());
      assertEquals("/ihmc/height_map/yolo/output/height_map_message", PerceptionAPI.YOLO_HEIGHT_MAP.getName());
      assertNotEquals("/ihmc/height_map/yolo/output", PerceptionAPI.YOLO_HEIGHT_MAP.getName());
      assertEquals("/ihmc/active_mapping/footstep_plan/output", ActiveMappingAPI.ACTIVE_MAPPING_FOOTSTEP_PLAN_OUTPUT.getName());
   }
}
