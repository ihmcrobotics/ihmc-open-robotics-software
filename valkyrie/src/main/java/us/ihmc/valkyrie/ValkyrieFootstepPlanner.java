package us.ihmc.valkyrie;

import java.io.IOException;
import java.net.URISyntaxException;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.log.LogTools;
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
         LogTools.error("Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }
   
   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }

   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      new ValkyrieFootstepPlanner();
   }
}
