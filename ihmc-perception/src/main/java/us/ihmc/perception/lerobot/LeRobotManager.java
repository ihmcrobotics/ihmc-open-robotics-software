package us.ihmc.perception.lerobot;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Topic;

public class LeRobotManager
{
   private static final ROS2Topic<?> LEROBOT = new ROS2Topic<>().withPrefix("lerobot");
   private static final ROS2Topic<std_msgs.msg.dds.String> COMMAND = LEROBOT.withSuffix("command").withType(std_msgs.msg.dds.String.class);

   private final String modelName;
   private final RepeatingTaskThread thread = new RepeatingTaskThread("LeRobotROS2Thread", this::update);
   private final ROS2Node ros2Node = new ROS2NodeBuilder().domainId(185).build("lerobot_java_side");
   private final ROS2Helper ros2 = new ROS2Helper(ros2Node);
   private final std_msgs.msg.dds.String command = new std_msgs.msg.dds.String();
   private boolean running = false;

   public LeRobotManager(String modelName)
   {
      this.modelName = modelName;

      thread.setFrequencyLimit(20.0);
      thread.startRepeating();
   }

   private void update()
   {
      command.setData(running ? modelName : "");
      ros2.publish(COMMAND, command);
   }

   public void setRunning(boolean running)
   {
      this.running = running;
   }

   public void destroy()
   {
      ros2Node.destroy();
      thread.kill();
   }

   public String getModelName()
   {
      return modelName;
   }
}
