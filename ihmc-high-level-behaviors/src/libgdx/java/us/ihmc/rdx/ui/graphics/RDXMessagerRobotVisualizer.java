package us.ihmc.rdx.ui.graphics;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.MessagerSyncedRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

public class RDXMessagerRobotVisualizer extends RDXMultiBodyGraphic
{
   private final MessagerSyncedRobotModel syncedRobot;
   private final ExceptionHandlingThreadScheduler scheduler;

   public RDXMessagerRobotVisualizer(DRCRobotModel robotModel,
                                     FullHumanoidRobotModel fullRobotModel,
                                     Messager messager,
                                     MessagerAPIFactory.Topic<RobotConfigurationData> topic)
   {
      super(robotModel.getSimpleRobotName() + " Robot Visualizer (Messager)");
      loadRobotModelAndGraphics(robotModel.getRobotDefinition(), fullRobotModel.getElevator());
      syncedRobot = new MessagerSyncedRobotModel(messager, topic, robotModel, fullRobotModel, robotModel.getSensorInformation());
      scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, 1, true);
   }

   public void update()
   {
      if (isRobotLoaded())
      {
         syncedRobot.update();
         super.update();
      }
   }

   public void destroy()
   {
      scheduler.shutdownNow();
      super.destroy();
   }
}
