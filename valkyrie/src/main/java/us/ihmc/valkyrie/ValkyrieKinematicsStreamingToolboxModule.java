package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class ValkyrieKinematicsStreamingToolboxModule extends KinematicsStreamingToolboxModule
{
   public ValkyrieKinematicsStreamingToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel, startYoVariableServer, pubSubImplementation);
   }

   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, false);
      boolean startYoVariableServer = true;
      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
      new ValkyrieKinematicsStreamingToolboxModule(robotModel, startYoVariableServer, pubSubImplementation);
   }
}
