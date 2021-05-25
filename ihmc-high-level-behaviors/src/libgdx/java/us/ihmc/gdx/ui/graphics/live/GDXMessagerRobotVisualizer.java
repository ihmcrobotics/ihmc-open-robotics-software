package us.ihmc.gdx.ui.graphics.live;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.MessagerSyncedRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.gdx.ui.graphics.GDXRobotGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

public class GDXMessagerRobotVisualizer extends GDXRobotGraphic
{
   private final MessagerSyncedRobotModel syncedRobot;
   private final ExceptionHandlingThreadScheduler scheduler;

   public GDXMessagerRobotVisualizer(DRCRobotModel robotModel,
                                     FullHumanoidRobotModel fullRobotModel,
                                     Messager messager,
                                     MessagerAPIFactory.Topic<RobotConfigurationData> topic)
   {
      super(robotModel, fullRobotModel);
      syncedRobot = new MessagerSyncedRobotModel(messager, topic, fullRobotModel);
      scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, 1, true);
   }

   public void update()
   {
      if (robotLoadedActivator.poll())
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
