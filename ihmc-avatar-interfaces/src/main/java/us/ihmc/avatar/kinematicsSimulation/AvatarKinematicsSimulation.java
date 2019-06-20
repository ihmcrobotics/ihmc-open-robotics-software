package us.ihmc.avatar.kinematicsSimulation;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.tools.functional.FunctionalTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class AvatarKinematicsSimulation
{
   private final DRCRobotModel robotModel;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final ExceptionHandlingThreadScheduler scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName());
   private ScheduledFuture<?> yoVariableServerScheduled;
   private double yoVariableServerTime = 0.0;
   private YoVariableServer yoVariableServer;
   private final AvatarKinematicsSimulationController controller;

   public AvatarKinematicsSimulation(DRCRobotModel robotModel, boolean startYoVariableServer)
   {
      this.robotModel = robotModel;

      controller = new AvatarKinematicsSimulationController(robotModel, 0.02, yoGraphicsListRegistry, registry);

      RobotConfigurationData initialConfigurationData = new RobotConfigurationData(); // TODO source this or create this?
      controller.updateRobotConfigurationData(initialConfigurationData);

      FunctionalTools.runIfTrue(startYoVariableServer, this::startYoVariableServer);

      controller.initialize();

      for (int i = 0; i < 1000; i++)
      {
         controller.updateInternal();
         controller.getFullRobotModel(); // TODO this is the output

         if (controller.isWalkingControllerResetDone())
            expectedNumberOfFrames++;

         scs.tickAndUpdate();

         if (controller.isDone())
            break;
      }
   }

   private void startYoVariableServer()
   {
      yoVariableServer = new YoVariableServer(getClass(), robotModel.getLogModelProvider(), new DataServerSettings(false), 0.01);
      yoVariableServer.setMainRegistry(registry, robotModel.createFullRobotModel().getElevator(), yoGraphicsListRegistry);
      ThreadTools.startAThread(() -> yoVariableServer.start(), getClass().getSimpleName() + "YoVariableServer");

      yoVariableServerScheduled = scheduler.schedule(this::yoVariableUpdateThread, 1, TimeUnit.MILLISECONDS);
   }

   private void yoVariableUpdateThread()
   {
      if (!Thread.interrupted())
      {
         yoVariableServerTime += Conversions.millisecondsToSeconds(1);
         yoVariableServer.update(Conversions.secondsToNanoseconds(yoVariableServerTime));
      }
   }
}
