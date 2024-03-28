package us.ihmc.avatar.networkProcessor.modules;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class ZeroPoseMockRobotConfigurationDataPublisherModule implements Runnable, CloseableAndDisposable
{
   private final ROS2Node ros2Node;
   private final ROS2PublisherBasics<RobotConfigurationData> publisher;
   private final FullHumanoidRobotModel fullRobotModel;
   private final ForceSensorDefinition[] forceSensorDefinitions;
   private long timeStamp = 0;

   private volatile boolean running = true;

   public ZeroPoseMockRobotConfigurationDataPublisherModule(final DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "ihmc_zero_pose_mock_node");
      fullRobotModel = robotModel.createFullRobotModel();
      forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();

      publisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(RobotConfigurationData.class).withTopic(ROS2Tools.getControllerOutputTopic(robotModel.getSimpleRobotName())));

      Thread t = new Thread(this);
      t.start();
   }

   public void sendMockRobotConfiguration(long totalNsecs)
   {
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel),
                                                                                           forceSensorDefinitions, imuDefinitions);

      for (int sensorNumber = 0; sensorNumber < imuDefinitions.length; sensorNumber++)
      {
         robotConfigurationData.getImuSensorData().add();
      }

      robotConfigurationData.setRobotMotionStatus(RobotMotionStatus.STANDING.toByte());
      robotConfigurationData.setWallTime(totalNsecs);
      robotConfigurationData.setMonotonicTime(totalNsecs);
      Vector3D translation = new Vector3D();
      Quaternion orientation = new Quaternion();
      robotConfigurationData.getRootPosition().set(translation);
      robotConfigurationData.getRootOrientation().set(orientation);

      publisher.publish(robotConfigurationData);
   }

   @Override
   public void run()
   {
      while (running)
      {
         sendMockRobotConfiguration(timeStamp);
         timeStamp += 250L * 1000000L;
         ThreadTools.sleep(250);
      }
   }

   @Override
   public void closeAndDispose()
   {
      running = false;
      ros2Node.destroy();
   }
}
