package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.PlanarRegionFootstepSnapper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ComponentBasedFootstepDataMessageGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ComponentBasedFootstepDataMessageGeneratorFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.List;

public class AvatarStepGeneratorThread implements AvatarControllerThreadInterface
{
   private final YoRegistry csgRegistry = new YoRegistry("csgRegistry");

   private final ComponentBasedFootstepDataMessageGenerator csg;
   private final FullHumanoidRobotModel fullRobotModel;

   private final HumanoidRobotContextData humanoidRobotContextData;
   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final YoBoolean firstTick = new YoBoolean("FirstTick", csgRegistry);
   private final YoLong timestampOffset = new YoLong("TimestampOffsetCSG", csgRegistry);
   private final YoDouble csgTime = new YoDouble("csgTime", csgRegistry);
   private final YoLong timestamp = new YoLong("TimestampCSG", csgRegistry);
   private final YoBoolean runCSG = new YoBoolean("RunCSG", csgRegistry);

   private final PlanarRegionFootstepSnapper planarRegionFootstepSnapper;
   private final CommandInputManager csgCommandInputManager;

   public AvatarStepGeneratorThread(ComponentBasedFootstepDataMessageGeneratorFactory csgPluginFactory,
                                    HumanoidRobotContextDataFactory contextDataFactory,
                                    StatusMessageOutputManager walkingOutputManager,
                                    CommandInputManager walkingCommandInputManager,
                                    DRCRobotModel drcRobotModel,
                                    double perceptionDt)
   {
      this.fullRobotModel = drcRobotModel.createFullRobotModel();
      contextDataFactory.setSensorDataContext(new SensorDataContext(fullRobotModel));
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      csg = csgPluginFactory.buildPlugin(humanoidReferenceFrames,
                                         perceptionDt,
                                         drcRobotModel.getWalkingControllerParameters(),
                                         walkingOutputManager,
                                         walkingCommandInputManager,
                                         null,
                                         null,
                                         csgTime);

      this.planarRegionFootstepSnapper = new PlanarRegionFootstepSnapper(csg.getContinuousStepGenerator(),
                                                                         drcRobotModel.getWalkingControllerParameters().getSteppingParameters(),
                                                                         csgRegistry);
      csg.getContinuousStepGenerator().setFootstepAdjustment(planarRegionFootstepSnapper);

      csgCommandInputManager = csgPluginFactory.getCSGCommandInputManager().getCommandInputManager();
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

   @Override
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

         // TODO should just poll the newest one
         List<PlanarRegionsListCommand> commands = csgCommandInputManager.pollNewCommands(PlanarRegionsListCommand.class);

         planarRegionFootstepSnapper.setPlanarRegions(commands.get(0));

         csg.update(csgTime.getValue());
         humanoidRobotContextData.setPerceptionRan(true);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return csgRegistry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   @Override
   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

}
