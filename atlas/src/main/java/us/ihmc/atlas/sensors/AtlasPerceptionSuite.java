package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.PerceptionSuite;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.ros2.Ros2Node;

public class AtlasPerceptionSuite extends PerceptionSuite
{
   private final DRCRobotModel robotModel;

   public AtlasPerceptionSuite(Messager messager)
   {
      super(messager);

      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);
   }

   @Override
   protected SLAMModule createSLAMModule() throws Exception
   {
      return AtlasSLAMModule.createIntraprocessModule(ros2Node, robotModel);
   }

   public static AtlasPerceptionSuite createIntraprocess() throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(PerceptionSuiteAPI.API,
                                                                NetworkPorts.PERCEPTION_SUITE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      return new AtlasPerceptionSuite(moduleMessager);
   }

}
