package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionSuite;

public class AtlasPerceptionSuite extends PerceptionSuite
{
   private final DRCRobotModel robotModel;

   public AtlasPerceptionSuite(Messager messager)
   {
      super(messager);

      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);
   }

   @Override
   protected SLAMModule createSLAMModule(Messager messager)
   {
      return AtlasSLAMModule.createIntraprocessModule(ros2Node, robotModel, messager);
   }

   public static AtlasPerceptionSuite createIntraprocess(Messager messager)
   {
      return new AtlasPerceptionSuite(messager);
   }

}
