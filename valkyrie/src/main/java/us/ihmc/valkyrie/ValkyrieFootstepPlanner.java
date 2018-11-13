package us.ihmc.valkyrie;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class ValkyrieFootstepPlanner
{
   public ValkyrieFootstepPlanner()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, false);
      
      
      tryToStartModule(() -> startFootstepModule(robotModel)); 
   }
   
   private void startFootstepModule(DRCRobotModel robotModel) throws IOException
   {
      //new FootstepPlanningToolboxModule(robotModel, null, true);
      new MultiStageFootstepPlanningModule(robotModel, null, true, PubSubImplementation.FAST_RTPS);
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

   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      ValkyrieFootstepPlanner footstepPlanner = new ValkyrieFootstepPlanner();
   }
}
