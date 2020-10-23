package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;

public class AtlasBehaviorUIAndModule
{
   public AtlasBehaviorUIAndModule(BehaviorUIRegistry behaviorRegistry)
   {
      CommunicationMode ros2CommunicationMode = CommunicationMode.INTERPROCESS;
      CommunicationMode messagerCommunicationMode = CommunicationMode.INTRAPROCESS;
      
      BehaviorModule behaviorModule = new BehaviorModule(behaviorRegistry, createRobotModel(), ros2CommunicationMode, messagerCommunicationMode);
      new BehaviorUI(behaviorRegistry, behaviorModule.getMessager(), createRobotModel(), ros2CommunicationMode.getPubSubImplementation());
   }
   
   private DRCRobotModel createRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
   }

   public static void main(String[] args)
   {
      new AtlasBehaviorUIAndModule(BehaviorUIRegistry.DEFAULT_BEHAVIORS);
   }
}
