package us.ihmc.avatar.footstepPlanning;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.commons.PrintTools;

import java.io.IOException;

public class AvatarFootstepPlanningModule
{
   public AvatarFootstepPlanningModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params)
   {
      tryToStartModule(() -> setupFootstepPlanningToolboxModule(robotModel, params));
   }

   private void setupFootstepPlanningToolboxModule(DRCRobotModel robotModel, DRCNetworkModuleParameters params) throws IOException
   {
      if (!params.isFootstepPlanningToolboxEnabled())
         return;

      new FootstepPlanningToolboxModule(robotModel, null, params.isFootstepPlanningToolboxVisualizerEnabled());
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         PrintTools.error(this, "Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }
}
