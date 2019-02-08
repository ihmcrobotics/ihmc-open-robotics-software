package us.ihmc.atlas.jfxvisualizer;

import controller_msgs.msg.dds.HighLevelStateMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;

/**
 * This class provides a visualizer for the remote footstep planner found in the footstep planner
 * toolbox. It allows users to view the resulting plans calculated by the toolbox. It also allows
 * the user to tune the planner parameters, and request a new plan from the planning toolboxs.
 */
public class AtlasRemoteFootstepPlannerUI extends Application
{
   private static final boolean launchPlannerToolbox = true;

   private SharedMemoryJavaFXMessager messager;
   private RemoteUIMessageConverter messageConverter;

   private FootstepPlannerUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
      DRCRobotModel previewRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
      messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      messageConverter = RemoteUIMessageConverter.createConverter(messager, drcRobotModel.getSimpleRobotName(), DomainFactory.PubSubImplementation.FAST_RTPS);

      messager.startMessager();

      ui = FootstepPlannerUI.createMessagerUI(primaryStage, messager, drcRobotModel.getFootstepPlannerParameters(),
                                              drcRobotModel.getVisibilityGraphsParameters(), drcRobotModel, previewRobotModel,
                                              drcRobotModel.getContactPointParameters(), drcRobotModel.getWalkingControllerParameters());
      ui.setRobotLowLevelMessenger(new AtlasLowLevelMessenger());
      ui.show();

      if (launchPlannerToolbox)
      {
         new MultiStageFootstepPlanningModule(drcRobotModel, null, false);
      }
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      messager.closeMessager();
      messageConverter.destroy();
      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }

   private class AtlasLowLevelMessenger implements RobotLowLevelMessenger
   {
      @Override
      public void sendFreezeRequest()
      {
         HighLevelStateMessage message = new HighLevelStateMessage();
         message.setHighLevelControllerName(HighLevelStateMessage.FREEZE_STATE);
         messager.submitMessage(FootstepPlannerMessagerAPI.HighLevelStateTopic, message);
      }

      @Override
      public void sendStandRequest()
      {
         HighLevelStateMessage message = new HighLevelStateMessage();
         message.setHighLevelControllerName(HighLevelStateMessage.STAND_PREP_STATE);
         messager.submitMessage(FootstepPlannerMessagerAPI.HighLevelStateTopic, message);
      }
   }
}