package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.WholeBodyMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JumpControlManagerFactory
{
   private final YoVariableRegistry registry;
   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private FeetJumpManager feetManager;
   private WholeBodyMomentumManager momentumManager;
   //private PlaneContactControlManager planeContactManager;
   private Map<String, RigidBodyControlManager> rigidBodyManagerMapByName = new HashMap<>();
   
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   public JumpControlManagerFactory(YoVariableRegistry registry)
   {
      this.registry = registry;
   }

   public WholeBodyMomentumManager getOrCreateWholeBodyMomentumManager()
   {
      if (momentumManager == null)
      {
         momentumManager = new WholeBodyMomentumManager(controllerToolbox, registry);
      }

      return momentumManager;
   }

//   public FeetJumpManager getOrCreateFeetJumpManager()
//   {
//      if (feetManager == null)
//      {
//         feetManager = new FeetJumpManager(controllerToolbox);
//      }
//      return feetManager;
//   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      return feedbackControlCommandList;
   }

   public void setHighLevelHumanoidControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
   }
}
