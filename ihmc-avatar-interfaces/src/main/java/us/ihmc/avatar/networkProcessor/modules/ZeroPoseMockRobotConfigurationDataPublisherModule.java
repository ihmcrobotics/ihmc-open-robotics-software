package us.ihmc.avatar.networkProcessor.modules;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class ZeroPoseMockRobotConfigurationDataPublisherModule implements Runnable
{
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_zero_pose_mock_node");
   private final IHMCROS2Publisher<RobotConfigurationData> publisher;
   private final FullHumanoidRobotModel fullRobotModel;
   private final ForceSensorDefinition[] forceSensorDefinitions;
   private long timeStamp = 0;

   public ZeroPoseMockRobotConfigurationDataPublisherModule(final DRCRobotModel robotModel)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();

      publisher = ROS2Tools.createPublisher(ros2Node, RobotConfigurationData.class,
                                            ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()));

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
      robotConfigurationData.getRootTranslation().set(translation);
      robotConfigurationData.getRootOrientation().set(orientation);

      publisher.publish(robotConfigurationData);
   }

   @Override
   public void run()
   {
      while (true)
      {
         sendMockRobotConfiguration(timeStamp);
         timeStamp += 250L * 1000000L;
         ThreadTools.sleep(250);
      }
   }

}
