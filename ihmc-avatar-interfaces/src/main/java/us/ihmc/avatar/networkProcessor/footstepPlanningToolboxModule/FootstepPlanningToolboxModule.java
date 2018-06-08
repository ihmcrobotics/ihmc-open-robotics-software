package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.FootstepPlanningRequestPacketPubSubType;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.TextToSpeechPacketPubSubType;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;

public class FootstepPlanningToolboxModule extends ToolboxModule
{
   private final FootstepPlanningToolboxController footstepPlanningToolboxController;

   public FootstepPlanningToolboxModule(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullHumanoidRobotModel, LogModelProvider modelProvider,
                                        boolean startYoVariableServer)
         throws IOException
   {
      super(fullHumanoidRobotModel, modelProvider, startYoVariableServer);
      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      footstepPlanningToolboxController = new FootstepPlanningToolboxController(drcRobotModel, fullHumanoidRobotModel, statusOutputManager, registry,
                                                                                yoGraphicsListRegistry,
                                                                                Conversions.millisecondsToSeconds(DEFAULT_UPDATE_PERIOD_MILLISECONDS));
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, new FootstepPlanningRequestPacketPubSubType(), "/ihmc/footstep_planning_request",
                                           s -> footstepPlanningToolboxController.processRequest(s.takeNextData()));
      footstepPlanningToolboxController.setTextToSpeechPublisher(ROS2Tools.createPublisher(realtimeRos2Node, new TextToSpeechPacketPubSubType(),
                                                                                           "/ihmc/text_to_speech"));
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return footstepPlanningToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(FootstepPlanningToolboxOutputStatus.class);
      return status;
   }

}
