package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.TriggerKickMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TriggerKickCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class KickDoorActionExecutor extends ActionNodeExecutor<KickDoorActionState, KickDoorActionDefinition>
{

   private final KickDoorActionState state;
   private final KickDoorActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ControllerStatusTracker controllerStatusTracker;
   private final WalkingControllerParameters walkingControllerParameters;
   private final FramePose3D solePose = new FramePose3D();
   private final RobotSide kickSide;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private boolean switchToKickControllerMessageSent = false;
   private boolean kickingMessageSent = false;
   private boolean switchToWalkControllerMessageSent = false;

   private Stopwatch stopwatch = new Stopwatch(); // Sorry Duncan
   private final TriggerKickCommand kickCommand = new TriggerKickCommand();
   private final FramePose3D doorHandlePose;

   public KickDoorActionExecutor(long id,
                                 CRDTInfo crdtInfo,
                                 WorkspaceResourceDirectory saveFileDirectory,
                                 ROS2ControllerHelper ros2ControllerHelper,
                                 ROS2SyncedRobotModel syncedRobot,
                                 ControllerStatusTracker controllerStatusTracker,
                                 ReferenceFrameLibrary referenceFrameLibrary,
                                 WalkingControllerParameters walkingControllerParameters)
   {
      super(new KickDoorActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();
      kickSide = definition.getSide();

      this.referenceFrameLibrary = referenceFrameLibrary;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
      this.walkingControllerParameters = walkingControllerParameters;

      doorHandlePose = new FramePose3D(referenceFrameLibrary.findFrameByName("RightDoorLeverHandle"));

      computeTriggerKickCommand(null,
                                definition.getKickHeight(),
                                definition.getKickTargetDistance(),
                                definition.getKickImpulse(),
                                definition.getPrekickWeightDistribution(), kickCommand);
   }

   /**
    * This update is running from the moment this behavior is added to the behavior tree, regardless of whether this behavior is being executed.
    */
   @Override
   public void update()
   {
      super.update();

      // add goal frame to definition or state (probs definition)
   }

   /**
    * Called at the beginning of execution of this behvior.
    */
   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      state.getExecutionState().setValue(KickDoorActionExecutionState.SWITCHING_HIGH_LEVEL_CONTROLLER_TO_KICK_CONTROLLER);
   }

   /**
    * This is only called while this behavior is being executed.
    */
   @Override
   public void updateCurrentlyExecuting()
   {
      switch (state.getExecutionState().getValue())
      {
         case STANDING ->
         {
            //Idle until the kick is requested
         }
         case SWITCHING_HIGH_LEVEL_CONTROLLER_TO_KICK_CONTROLLER ->
         {
            stopwatch.start();
            if (!switchToKickControllerMessageSent)
            {
               // The kicking controller must be added with a boolean in the controller factory.
               changeHighLevelState(HighLevelControllerName.CUSTOM1);
               switchToKickControllerMessageSent = true;
            }

            if (stopwatch.lapElapsed() >= 0.2)
            {
               state.getExecutionState().setValue(KickDoorActionExecutionState.EXECUTING_KICKING);
            }
         }
         case EXECUTING_KICKING ->
         {
            //Execute the kick
            state.setIsExecuting(true);
            stopwatch.reset();

            if (!kickingMessageSent)
            {
               ros2ControllerHelper.publishToController(kickCommand);
               kickingMessageSent = true;
            }

            if (stopwatch.lapElapsed() >= 2.0)
            {
               state.getExecutionState().setValue(KickDoorActionExecutionState.KICK_COMPLETED);
               state.setIsExecuting(false);
            }
         }
         case SWITCHING_HIGH_LEVEL_CONTROLLER_TO_WALKING_CONTROLLER ->
         {
            stopwatch.reset();
            if (!switchToWalkControllerMessageSent)
            {
               changeHighLevelState(HighLevelControllerName.WALKING);
               switchToWalkControllerMessageSent = true;
            }

            if (stopwatch.lapElapsed() >= 0.2)
            {
               state.getExecutionState().setValue(KickDoorActionExecutionState.STANDING);
            }
         }
      }
   }

   private void changeHighLevelState(HighLevelControllerName highLevelControllerName)
   {
      // Switch the high level state machine to the desired controller
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(highLevelControllerName.toByte());
      ros2ControllerHelper.publishToController(highLevelStateMessage);
   }

   public void computeTriggerKickCommand(FramePose3DReadOnly doorHandlePose,
                                         double kickHeight,
                                         double desiredKickDistance,
                                         double desiredKickImpulse,
                                         double desiredPrekickWeightDistribution,
                                         TriggerKickCommand commandToPack)
   {
      TriggerKickMessage kickMessage = new TriggerKickMessage();
      kickMessage.setRobotSide(kickSide.toByte());
      kickMessage.setKickHeight(kickHeight);
      kickMessage.setKickImpulse(desiredKickImpulse);
      kickMessage.setKickTargetDistance(desiredKickDistance);
      kickMessage.setPrekickWeightDistribution(desiredPrekickWeightDistribution);
      commandToPack.setFromMessage(kickMessage);
   }
}
