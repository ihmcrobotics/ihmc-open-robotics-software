package us.ihmc.avatar.ros.networkTest.profiles;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.ros.networkTest.ROS2NetworkTestMachine;
import us.ihmc.avatar.ros.networkTest.ROS2NetworkTestProfile;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.List;

/**
 * This class creates random data robot configuration data with timestamps
 * and publishes it to 3 computers.
 *
 * TODO: Create map list of expected traffic and profile should run with that.
 */
public class RobotConfigurationData100HzNetworkTestProfile extends ROS2NetworkTestProfile
{
   public static final ROS2QosProfile QOS_PROFILE = ROS2QosProfile.DEFAULT();

   private static final ROS2Topic<RobotConfigurationData> TOPIC = ROS2Tools.IHMC_ROOT.withModule("rcd100Hz").withType(RobotConfigurationData.class);

   public static final double PUBLISH_FREQUENCY = 100.0;
   public static final double EXPERIMENT_DURATION = 100.0;

   private final YoRegistry yoRegistry = new YoRegistry(getMachineName() + getClass().getSimpleName());

   private YoLong ocuSent;

   @Override
   public List<ROS2NetworkTestMachine> getMachines()
   {
      return null;
   }

   @Override
   public void runExperiment()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return null;
   }

   @Override
   public void destroy()
   {

   }

   @Override
   public List<String[]> getGraphsToSetup()
   {
      return null;
   }
}
