package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.WholeBodyMomentumManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JumpControlManagerFactory
{
   private final YoVariableRegistry registry;
   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private FeetJumpManager feetManager;
   //private final WholeBodyAngularVelocityManager angularVelocityManager;
   private WholeBodyMomentumManager momentumManager;
  
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   
   public JumpControlManagerFactory(YoVariableRegistry registry)
   {
      this.registry = registry;
   }
   
   public WholeBodyMomentumManager getOrCreateWholeBodyMomentumManager()
   {
      if(momentumManager == null)
      {
         momentumManager = new WholeBodyMomentumManager(controllerToolbox, registry);
      }
      
      return momentumManager;
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      return feedbackControlCommandList;
   }
   
   public FeetJumpManager getOrCreateFeetJumpManager()
   {
      if(feetManager == null)
      {
         feetManager = new FeetJumpManager(controllerToolbox);
      }
      return feetManager;
   }

   public void setHighLevelHumanoidControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
   }
}
