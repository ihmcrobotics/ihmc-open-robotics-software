package us.ihmc.valkyrie.externalForceEstimation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

public class ValkyrieExternalForceEstimationModule extends ExternalForceEstimationToolboxModule
{
   public static final ValkyrieRobotVersion version = ValkyrieRosControlController.VERSION; // ValkyrieRobotVersion.ARM_MASS_SIM; //

   public ValkyrieExternalForceEstimationModule(DRCRobotModel robotModel, boolean startYoVariableServer, RealtimeROS2Node ros2Node)
   {
      super(robotModel, startYoVariableServer, ros2Node);
   }

   public ValkyrieExternalForceEstimationModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel, startYoVariableServer, pubSubImplementation);
   }
   
   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, version);

      boolean startYoVariableServer = false;
      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
      new ExternalForceEstimationToolboxModule(robotModel, startYoVariableServer, pubSubImplementation);
   }
}
