package us.ihmc.atlas.contactEstimation;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeROS2Node;

public class AtlasContactEstimationModule extends ExternalForceEstimationToolboxModule
{
   public static final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;

   public AtlasContactEstimationModule(DRCRobotModel robotModel, boolean startYoVariableServer, RealtimeROS2Node ros2Node)
   {
      super(robotModel, new AtlasMultiContactCollisionModel(robotModel.getJointMap()), startYoVariableServer, ros2Node);
   }

   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS);
      boolean startYoVariableServer = true;
      DomainFactory.PubSubImplementation pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
      new ExternalForceEstimationToolboxModule(robotModel,
                                               new AtlasMultiContactCollisionModel(robotModel.getJointMap()),
                                               startYoVariableServer,
                                               pubSubImplementation);
   }
}
