package us.ihmc.atlas.behaviors;

import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.tools.processManagement.JavaProcessManager;

public class AtlasLookAndStepBehaviorUIAndModule
{
   public static void main(String[] args)
   {
      JavaProcessManager manager = new JavaProcessManager();
      manager.runOrRegister("AtlasBehaviorUIAndModule", () -> new AtlasBehaviorUIAndModule(BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION)));
//      manager.runOrRegister("RealsenseSLAM", () -> new AtlasSLAMBasedREAStandaloneLauncher(false, PubSubImplementation.FAST_RTPS));
//      manager.runOrRegister("LidarREA", () -> new LidarBasedREAStandaloneLauncher());
//      manager.runOrRegister("LidarREA", () -> new RemoteLidarBasedREAModuleLauncher());
      manager.spawnProcesses(AtlasLookAndStepBehaviorUIAndModule.class, args);
   }
}
