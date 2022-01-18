package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ComponentBasedFootstepDataMessageGenerator implements HighLevelHumanoidControllerPlugin
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ContinuousStepGenerator continuousStepGenerator;
   private final HeadingAndVelocityEvaluationScript script;

   public ComponentBasedFootstepDataMessageGenerator(CommonHumanoidReferenceFrames referenceFrames,
                                                     double controlDT,
                                                     DoubleProvider timeProvider,
                                                     WalkingControllerParameters walkingControllerParameters,
                                                     StatusMessageOutputManager statusMessageOutputManager,
                                                     CommandInputManager commandInputManager,
                                                     SideDependentList<? extends ContactableBody> contactableFeet,
                                                     boolean useHeadingAndVelocityScript,
                                                     HeightMap heightMapForFootstepZ,
                                                     HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters,
                                                     YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      continuousStepGenerator = new ContinuousStepGenerator(registry);
      continuousStepGenerator.setFootstepStatusListener(statusMessageOutputManager);
      continuousStepGenerator.setFrameBasedFootPoseProvider(referenceFrames.getSoleZUpFrames());
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.setFootstepMessenger(commandInputManager::submitMessage);
      if (yoGraphicsListRegistry != null)
         continuousStepGenerator.setupVisualization(contactableFeet, yoGraphicsListRegistry);
      if (heightMapForFootstepZ != null)
         continuousStepGenerator.setHeightMapBasedFootstepAdjustment(heightMapForFootstepZ);

      if (useHeadingAndVelocityScript)
      {
         script = new HeadingAndVelocityEvaluationScript(controlDT, timeProvider, headingAndVelocityEvaluationScriptParameters, registry);
         continuousStepGenerator.setDesiredTurningVelocityProvider(script.getDesiredTurningVelocityProvider());
         continuousStepGenerator.setDesiredVelocityProvider(script.getDesiredVelocityProvider());
      }
      else
      {
         script = null;
         continuousStepGenerator.setYoComponentProviders();
      }
   }

   @Override
   public void update(double time)
   {
      if (script != null)
         script.update(time);
      continuousStepGenerator.update(time);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   public static void newFactory()
   {
      newFactory(false, null, null);
   }

   public static void newFactory(boolean useHeadingAndVelocityScript, HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters)
   {
      newFactory(useHeadingAndVelocityScript, null, headingAndVelocityEvaluationScriptParameters);
   }

   public static HighLevelHumanoidControllerPluginFactory newFactory(boolean useHeadingAndVelocityScript,
                                                                     HeightMap heightMapForFootstepZ,
                                                                     HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters)
   {
      return new HighLevelHumanoidControllerPluginFactory()
      {
         @Override
         public HighLevelHumanoidControllerPlugin buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper)
         {
            HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();
            CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
            double controlDT = controllerToolbox.getControlDT();
            WalkingControllerParameters walkingControllerParameters = controllerFactoryHelper.getWalkingControllerParameters();
            StatusMessageOutputManager statusMessageOutputManager = controllerFactoryHelper.getStatusMessageOutputManager();
            CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();
            YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
            SideDependentList<? extends ContactableBody> contactableFeet = controllerToolbox.getContactableFeet();
            DoubleProvider timeProvider = controllerToolbox.getYoTime();
            return new ComponentBasedFootstepDataMessageGenerator(referenceFrames,
                                                                  controlDT,
                                                                  timeProvider,
                                                                  walkingControllerParameters,
                                                                  statusMessageOutputManager,
                                                                  commandInputManager,
                                                                  contactableFeet,
                                                                  useHeadingAndVelocityScript,
                                                                  heightMapForFootstepZ,
                                                                  headingAndVelocityEvaluationScriptParameters,
                                                                  yoGraphicsListRegistry);
         }
      };
   }
}
