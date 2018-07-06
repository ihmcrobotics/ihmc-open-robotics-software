package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedControlManagerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedPhysicalProperties physicalProperties;
   private final QuadrupedControllerToolbox toolbox;
   private final YoGraphicsListRegistry graphicsListRegistry;

   private QuadrupedFeetManager feetManager;
   private QuadrupedBodyOrientationManager bodyOrientationManager;
   private QuadrupedBalanceManager balanceManager;
   private QuadrupedJointSettingsManager jointSpaceManager;

   public QuadrupedControlManagerFactory(QuadrupedControllerToolbox toolbox, QuadrupedPhysicalProperties physicalProperties,
                                         YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;
      this.physicalProperties = physicalProperties;
      this.graphicsListRegistry = graphicsListRegistry;

      parentRegistry.addChild(registry);
   }

   public QuadrupedFeetManager getOrCreateFeetManager()
   {
      if (feetManager != null)
         return feetManager;

      feetManager = new QuadrupedFeetManager(toolbox, graphicsListRegistry, registry);
      return feetManager;
   }

   public QuadrupedBodyOrientationManager getOrCreateBodyOrientationManager()
   {
      if (bodyOrientationManager != null)
         return bodyOrientationManager;

      bodyOrientationManager = new QuadrupedBodyOrientationManager(toolbox, registry);
      return bodyOrientationManager;
   }

   public QuadrupedBalanceManager getOrCreateBalanceManager()
   {
      if (balanceManager != null)
         return balanceManager;

      balanceManager = new QuadrupedBalanceManager(toolbox, physicalProperties, registry, toolbox.getRuntimeEnvironment().getGraphicsListRegistry());
      return balanceManager;
   }

   public QuadrupedJointSettingsManager getOrCreateJointSpaceManager()
   {
      if (jointSpaceManager != null)
         return jointSpaceManager;

      jointSpaceManager = new QuadrupedJointSettingsManager(toolbox, registry);
      return jointSpaceManager;
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();

      if (feetManager != null)
         ret.addCommandList(feetManager.createFeedbackControlTemplate());
      if (bodyOrientationManager != null)
         ret.addCommand(bodyOrientationManager.createFeedbackControlTemplate());
      if (jointSpaceManager != null)
         ret.addCommand(jointSpaceManager.createFeedbackControlTemplate());

      return ret;
   }
}
