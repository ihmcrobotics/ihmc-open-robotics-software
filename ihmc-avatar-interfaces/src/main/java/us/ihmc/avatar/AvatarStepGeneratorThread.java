package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Arrays;

public class AvatarStepGeneratorThread implements SCS2YoGraphicHolder
{
   private final YoRegistry csgRegistry = new YoRegistry("csgRegistry");

   private final HumanoidSteppingManager steppingManager;
   private final FullHumanoidRobotModel fullRobotModel;

   private final HumanoidRobotContextData humanoidRobotContextData;
   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final YoBoolean firstTick = new YoBoolean("FirstTick", csgRegistry);
   private final YoLong timestampOffset = new YoLong("TimestampOffsetCSG", csgRegistry);
   private final YoDouble csgTime = new YoDouble("csgTime", csgRegistry);
   private final YoLong timestamp = new YoLong("TimestampCSG", csgRegistry);
   private final YoBoolean runCSG = new YoBoolean("RunCSG", csgRegistry);

   private final StepGeneratorCommandInputManager csgCommandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final HumanoidSteppingEnvironmentalConstraints environmentalConstraints;

   public AvatarStepGeneratorThread(HumanoidRobotContextDataFactory contextDataFactory,
                                    StatusMessageOutputManager controllerOutputManager,
                                    CommandInputManager controllerCommandInputManager,
                                    DRCRobotModel drcRobotModel,
                                    FootstepAdjustment footstepAdjustment,
                                    AsyncROS2Node ros2Node)
   {
      this.fullRobotModel = drcRobotModel.createFullRobotModel();

      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(fullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolderForController = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      CenterOfMassDataHolder centerOfMassDataHolderForController = new CenterOfMassDataHolder();
      CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator = new CenterOfPressureDataHolder(fullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(fullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForController);
      contextDataFactory.setCenterOfMassDataHolder(centerOfMassDataHolderForController);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderForEstimator);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      contextDataFactory.setSensorDataContext(new SensorDataContext(fullRobotModel));
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      steppingManager = new HumanoidSteppingManager(fullRobotModel,
                                                    humanoidReferenceFrames,
                                                    drcRobotModel::getStepGeneratorDT,
                                                    drcRobotModel.getWalkingControllerParameters(),
                                                    controllerOutputManager,
                                                    controllerCommandInputManager,
                                                    null,
                                                    csgTime);
      csgCommandInputManager = steppingManager.getStepGeneratorCommandInputManager();
      statusMessageOutputManager = steppingManager.getStatusMessageOutputManager();
      csgCommandInputManager.addHeightMapCommandConsumer(controllerCommandInputManager::submitCommand);

      // create the callback listeners for the planar regions in the stepping plugin
      if (ros2Node != null)
         steppingManager.createStepGeneratorNetworkSubscriber(drcRobotModel.getSimpleRobotName(), ros2Node);

      if (footstepAdjustment != null)
      {
         steppingManager.setFootstepAdjustment(footstepAdjustment);
         environmentalConstraints = null;
      }
      else
      {
         environmentalConstraints = new HumanoidSteppingEnvironmentalConstraints(drcRobotModel.getContactPointParameters(),
                                                                                 drcRobotModel.getWalkingControllerParameters()
                                                                                              .getSteppingParametersForStepGeneration());
         environmentalConstraints.setSnapToHeightMap(true);
         // sets up the environmental constraint manager as a planar region consumer in the input manager
         csgCommandInputManager.addHeightMapCommandConsumer(environmentalConstraints);
         // Adds functions that adjust the footholds based on the environment.
         steppingManager.setFootstepAdjustment(environmentalConstraints.getFootstepAdjustment());
         // Adds checkers for footholds based on the environment
         for (FootstepValidityIndicator footstepValidityIndicator : environmentalConstraints.getFootstepValidityIndicators())
            steppingManager.addFootstepValidityIndicator(footstepValidityIndicator);

         // clear the environment at the beginning of every update
         steppingManager.addUpdatable(environmentalConstraints);
         csgRegistry.addChild(environmentalConstraints.getRegistry());
      }
      csgRegistry.addChild(steppingManager.getRegistry());

      ParameterLoaderHelper.loadParameters(this, drcRobotModel, csgRegistry);
   }

   public DoubleProvider getYoTime()
   {
      return csgTime;
   }

   public void initialize()
   {
      firstTick.set(true);
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);
   }

   private void runOnFirstTick()
   {
   }

   public void run()
   {
      runCSG.set(humanoidRobotContextData.getEstimatorRan());
      if (!runCSG.getValue())
      {
         return;
      }

      try
      {
         HumanoidRobotContextTools.updateRobot(fullRobotModel, humanoidRobotContextData.getProcessedJointData());
         humanoidReferenceFrames.updateFrames();

         timestamp.set(humanoidRobotContextData.getTimestamp());
         if (firstTick.getValue())
         {
            // Record this to have time start at 0.0 on the real robot for viewing pleasure.
            timestampOffset.set(timestamp.getValue());
         }
         csgTime.set(Conversions.nanosecondsToSeconds(timestamp.getValue() - timestampOffset.getValue()));

         if (firstTick.getValue())
         {
            runOnFirstTick();
            firstTick.set(false);
         }

         steppingManager.update(csgTime.getValue());
         humanoidRobotContextData.setPerceptionRan(true);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public HumanoidSteppingManager getSteppingManager()
   {
      return steppingManager;
   }

   public YoRegistry getYoVariableRegistry()
   {
      return csgRegistry;
   }

   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(steppingManager.getSCS2YoGraphics());
      if (environmentalConstraints != null)
         group.addChild(environmentalConstraints.getSCS2YoGraphics());
      return group.isEmpty() ? null : group;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

   public StepGeneratorCommandInputManager getCsgCommandInputManager()
   {
      return csgCommandInputManager;
   }

   public StatusMessageOutputManager getStatusOutputManager()
   {
      return statusMessageOutputManager;
   }

   public void destroy()
   {
      csgCommandInputManager.destroy();
   }
}
