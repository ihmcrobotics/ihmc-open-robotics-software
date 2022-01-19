package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ComponentBasedFootstepDataMessageGeneratorFactory implements HighLevelHumanoidControllerPluginFactory
{
   private final RequiredFactoryField<YoRegistry> registryField = new RequiredFactoryField<>("registry");

   private final OptionalFactoryField<Boolean> useHeadingAndVelocityScriptField = new OptionalFactoryField<>("useHeadingAndVelocityScript", false);
   private final OptionalFactoryField<HeadingAndVelocityEvaluationScriptParameters> headingAndVelocityEvaluationScriptParametersField = new OptionalFactoryField<>("headingAndVelocityEvaluationScriptParameters");
   private final OptionalFactoryField<HeightMap> heightMapField = new OptionalFactoryField<>("heightMap");

   public ComponentBasedFootstepDataMessageGeneratorFactory()
   {
   }

   public ComponentBasedFootstepDataMessageGeneratorFactory setRegistry()
   {
      return setRegistry(ComponentBasedFootstepDataMessageGenerator.class.getSimpleName());
   }

   public ComponentBasedFootstepDataMessageGeneratorFactory setRegistry(String name)
   {
      registryField.set(new YoRegistry(name));
      return this;
   }

   public void setHeightMap(HeightMap heightMap)
   {
      heightMapField.set(heightMap);
   }

   public void setUseHeadingAndVelocityScript(boolean useHeadingAndVelocityScript)
   {
      useHeadingAndVelocityScriptField.set(useHeadingAndVelocityScript);
   }

   public void setHeadingAndVelocityEvaluationScriptParameters(HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters)
   {
      this.headingAndVelocityEvaluationScriptParametersField.set(headingAndVelocityEvaluationScriptParameters);
   }

   @Override
   public ComponentBasedFootstepDataMessageGenerator buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      double controlDT = controllerToolbox.getControlDT();
      WalkingControllerParameters walkingControllerParameters = controllerFactoryHelper.getWalkingControllerParameters();
      StatusMessageOutputManager statusMessageOutputManager = controllerFactoryHelper.getStatusMessageOutputManager();
      CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      SideDependentList<? extends ContactableBody> contactableFeet = controllerToolbox.getContactableFeet();
      DoubleProvider timeProvider = controllerToolbox.getYoTime();

      ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator(registryField.get());

      continuousStepGenerator.setFootstepStatusListener(statusMessageOutputManager);
      continuousStepGenerator.setFrameBasedFootPoseProvider(referenceFrames.getSoleZUpFrames());
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.setFootstepMessenger(commandInputManager::submitMessage);

      List<Updatable> updatables = new ArrayList<>();

      if (yoGraphicsListRegistry != null)
         continuousStepGenerator.setupVisualization(contactableFeet, yoGraphicsListRegistry);
      if (heightMapField.hasValue() && heightMapField.get() != null)
         continuousStepGenerator.setHeightMapBasedFootstepAdjustment(heightMapField.get());

      if (useHeadingAndVelocityScriptField.get())
      {
         HeadingAndVelocityEvaluationScript script = new HeadingAndVelocityEvaluationScript(controlDT,
                                                                                            timeProvider,
                                                                                            headingAndVelocityEvaluationScriptParametersField.get(),
                                                                                            registryField.get());
         continuousStepGenerator.setDesiredTurningVelocityProvider(script.getDesiredTurningVelocityProvider());
         continuousStepGenerator.setDesiredVelocityProvider(script.getDesiredVelocityProvider());
         updatables.add(script);
      }
      else
      {
         continuousStepGenerator.setYoComponentProviders();
      }

      ComponentBasedFootstepDataMessageGenerator plugin = new ComponentBasedFootstepDataMessageGenerator(continuousStepGenerator,
                                                                                                         updatables,
                                                                                                         registryField.get());
      FactoryTools.disposeFactory(this);
      return plugin;
   }

   public static class CSGCommandInputManager implements Updatable
   {
      private final CommandInputManager commandInputManager = new CommandInputManager(supportedCommands());

      public CSGCommandInputManager()
      {
      }

      public List<Class<? extends Command<?, ?>>> supportedCommands()
      {
         List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
         commands.add(ContinuousStepGeneratorParametersCommand.class);
         commands.add(ContinuousStepGeneratorInputCommand.class);
         return commands;
      }

      @Override
      public void update(double time)
      {
      }
   }
}
