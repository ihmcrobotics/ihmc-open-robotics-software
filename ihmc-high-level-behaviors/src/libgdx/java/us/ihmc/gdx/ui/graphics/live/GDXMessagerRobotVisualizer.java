package us.ihmc.gdx.ui.graphics.live;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.MessagerSyncedRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.gdx.ui.graphics.GDXRobotModelGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

public class GDXMessagerRobotVisualizer extends GDXRobotModelGraphic
{
   private final MessagerSyncedRobotModel syncedRobot;
   private final ExceptionHandlingThreadScheduler scheduler;

   public GDXMessagerRobotVisualizer(DRCRobotModel robotModel,
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
