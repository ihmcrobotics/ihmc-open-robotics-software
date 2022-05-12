package us.ihmc.avatar;

import us.ihmc.avatar.AvatarControllerThreadInterface;
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
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.nadia.operation.joystickPlugin.NadiaRCJoystickPlugin;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

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

   private final PlanarRegionFootstepSnapper planarRegionFootstepSnapper = new PlanarRegionFootstepSnapper();
   private final CommandInputManager walkingCommandInputmanager;
   private final CommandInputManager csgCommandInputManager;

   public AvatarStepGeneratorThread(ComponentBasedFootstepDataMessageGeneratorFactory csgPluginFactory,
                                    HumanoidRobotContextDataFactory contextDataFactory,
                                    StatusMessageOutputManager walkingOutputManager,
                                    CommandInputManager walkingCommandInputManager,
                                    DRCRobotModel drcRobotModel,
                                    RealtimeROS2Node ros2Node,
                                    double perceptionDt)
   {
      this.fullRobotModel = drcRobotModel.createFullRobotModel();
      this.walkingCommandInputmanager = walkingCommandInputManager;
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

   private FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide)
   {
      FramePose3D adjustedBasedOnStanceFoot = new FramePose3D();
      adjustedBasedOnStanceFoot.getPosition().set(footstepPose.getPosition());
      adjustedBasedOnStanceFoot.setZ(continuousStepGenerator.getCurrentSupportFootPose().getZ());
      adjustedBasedOnStanceFoot.getOrientation().set(footstepPose.getOrientation());

      if (planarRegionsList.get() != null)
      {
         FramePose3D wiggledPose = new FramePose3D(adjustedBasedOnStanceFoot);
         footPolygonToWiggle.set(footPolygons.get(footSide));
         try
         {
            snapAndWiggleSingleStep.snapAndWiggle(wiggledPose, footPolygonToWiggle, forwardVelocity.getValue() > 0.0);
            if (wiggledPose.containsNaN())
               return adjustedBasedOnStanceFoot;
         }
         catch (SnapAndWiggleSingleStep.SnappingFailedException e)
         {
            /*
             * It's fine if the snap & wiggle fails, can be because there no planar regions around the footstep.
             * Let's just keep the adjusted footstep based on the pose of the current stance foot.
             */
         }
         return wiggledPose;
      }
      else
      {
         return adjustedBasedOnStanceFoot;
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
