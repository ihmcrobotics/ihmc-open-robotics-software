package us.ihmc.valkyrie.externalForceEstimation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieExternalForceEstimationModule extends ExternalForceEstimationToolboxModule
{
   public ValkyrieExternalForceEstimationModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel, startYoVariableServer, pubSubImplementation);
   }
   
   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, true);

      boolean startYoVariableServer = true;
      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
      new ExternalForceEstimationToolboxModule(robotModel, startYoVariableServer, pubSubImplementation);
   }
}
