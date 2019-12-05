package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasKinematicSimulation
{
   public static void create(AtlasRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      HumanoidKinematicsSimulation.create(robotModel, createYoVariableServer, pubSubImplementation);
   }

   public static void main(String[] args)
   {
      AtlasKinematicSimulation.create(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false),
                                      false,
                                      PubSubImplementation.FAST_RTPS);
   }
}
