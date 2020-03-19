package us.ihmc.atlas.sensors;

import java.io.File;
import java.io.IOException;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;

public class AtlasSLAMModule extends SLAMModule
{
   public AtlasSLAMModule(Messager messager, DRCRobotModel drcRobotModel, File configurationFile)
   {
      super(messager, configurationFile);

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           RobotConfigurationData.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(drcRobotModel.getSimpleRobotName()),
                                           this::updateStationaryStatus);

      String generateTopicName = ControllerAPIDefinition.getPublisherTopicNameGenerator(drcRobotModel.getSimpleRobotName())
                                                        .generateTopicName(StampedPosePacket.class);
      estimatedPelvisPublisher = ROS2Tools.createPublisher(ros2Node, StampedPosePacket.class, generateTopicName);
      sensorPoseToPelvisTransformer = new RigidBodyTransform(AtlasSensorInformation.transformPelvisToDepthCamera);
      sensorPoseToPelvisTransformer.invert();
   }

   public static AtlasSLAMModule createIntraprocessModule(DRCRobotModel drcRobotModel, String configurationFilePath) throws Exception
   {
      KryoMessager messager = KryoMessager.createIntraprocess(REAModuleAPI.API,
                                                              NetworkPorts.REA_MODULE_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      messager.setAllowSelfSubmit(true);
      messager.startMessager();

      File configurationFile = new File(configurationFilePath);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }

      return new AtlasSLAMModule(messager, drcRobotModel, configurationFile);
   }
}
