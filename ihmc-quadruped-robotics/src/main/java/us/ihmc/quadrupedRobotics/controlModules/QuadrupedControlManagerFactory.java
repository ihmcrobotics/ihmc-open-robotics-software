package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedControlManagerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedForceControllerToolbox toolbox;
   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final YoGraphicsListRegistry graphicsListRegistry;

   private QuadrupedFeetManager feetManager;
   private QuadrupedBodyOrientationManager bodyOrientationManager;
   private QuadrupedBalanceManager balanceManager;
   private QuadrupedJointSpaceManager jointSpaceManager;

   public QuadrupedControlManagerFactory(QuadrupedForceControllerToolbox toolbox, QuadrupedPhysicalProperties physicalProperties, GlobalDataProducer globalDataProducer,
                                         YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;
      this.postureProvider = new QuadrupedPostureInputProvider(physicalProperties, globalDataProducer, registry);
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

      balanceManager = new QuadrupedBalanceManager(toolbox, postureProvider, registry, toolbox.getRuntimeEnvironment().getGraphicsListRegistry());
      return balanceManager;
   }

   public QuadrupedJointSpaceManager getOrCreateJointSpaceManager()
   {
      if (jointSpaceManager != null)
         return jointSpaceManager;

      jointSpaceManager = new QuadrupedJointSpaceManager(toolbox, registry);
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
