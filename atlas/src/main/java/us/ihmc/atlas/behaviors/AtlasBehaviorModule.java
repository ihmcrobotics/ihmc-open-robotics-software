package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.log.LogTools;

public class AtlasBehaviorModule
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private final BehaviorModule behaviorModule;

   public AtlasBehaviorModule()
   {
      LogTools.info("Creating behavior module");
      Runtime.getRuntime().addShutdownHook(new Thread(this::shutdown, "Cleanup"));

      BehaviorRegistry registry = BehaviorRegistry.DEFAULT_BEHAVIORS;
      registry.activateRegistry();
      boolean enableROS1 = true;
      behaviorModule = new BehaviorModule(registry,
                                          createRobotModel(),
                                          CommunicationMode.INTERPROCESS,
                                          CommunicationMode.INTERPROCESS,
                                          enableROS1);
   }

   private void shutdown() // add cleanup actions here
   {
      behaviorModule.destroy();
   }

   private AtlasRobotModel createRobotModel()
   {
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);
   }

   /** To run remotely */
   public static void main(String[] args)
   {
      new AtlasBehaviorModule();
   }
}
