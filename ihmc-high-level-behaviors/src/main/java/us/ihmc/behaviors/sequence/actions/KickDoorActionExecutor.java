package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.TriggerKickMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TriggerKickCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.NonWallTimer;
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
   private boolean squareUpFootstepsSent = false;

   private final NonWallTimer stopwatch = new NonWallTimer();
   private final TriggerKickCommand kickCommand = new TriggerKickCommand();
   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

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

      computeTriggerKickCommand(definition.getKickHeight(),
                                definition.getKickTargetDistance(),
                                definition.getKickImpulse(),
                                definition.getPrekickWeightDistribution(),
                                kickCommand);
   }

   /**
    * This update is running from the moment this behavior is added to the behavior tree, regardless of whether this behavior is being executed.
    */
   @Override
   public void update()
   {
      super.update();

      stopwatch.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      // add goal frame to definition or state (probs definition)
   }

   /**
    * Called at the beginning of execution of this behvior.
    */
   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      state.getExecutionState().setValue(KickDoorActionExecutionState.SWITCHING_TO_KICK_CONTROLLER);
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
         case SWITCHING_TO_KICK_CONTROLLER ->
         {
            if (!switchToKickControllerMessageSent)
            {
               // The kicking controller must be added with a boolean in the controller factory.
               state.getLogger().info("Switching to kick controller message.");
               changeHighLevelState(HighLevelControllerName.CUSTOM1);
               switchToKickControllerMessageSent = true;
               stopwatch.reset();
            }

            if (stopwatch.getElapsedTime() >= 0.2)
            {
               stopwatch.reset();
               state.getExecutionState().setValue(KickDoorActionExecutionState.KICKING);
            }
         }
         case KICKING ->
         {
            //Execute the kick
            state.setIsExecuting(true);

            if (!kickingMessageSent)
            {
               stopwatch.reset();
               kickingMessageSent = true;
               state.getLogger().info("Executing kick.");
               //               ros2ControllerHelper.publishToController(kickCommand);
            }

            if (stopwatch.getElapsedTime() >= 2.0)
            {
               state.getExecutionState().setValue(KickDoorActionExecutionState.SWITCHING_TO_WALKING_CONTROLLER);
               stopwatch.reset();
            }
         }
         case SWITCHING_TO_WALKING_CONTROLLER ->
         {
            if (!switchToWalkControllerMessageSent)
            {
               stopwatch.reset();
               state.getLogger().info("Sending switch to walking controller message.");
               changeHighLevelState(HighLevelControllerName.WALKING);
               switchToWalkControllerMessageSent = true;
            }

            if (stopwatch.getElapsedTime() >= 0.2)
            {
               state.getExecutionState().setValue(KickDoorActionExecutionState.SQUARING_UP);
               stopwatch.reset();
            }
         }
         case SQUARING_UP ->
         {
            if (!squareUpFootstepsSent)
            {
               stopwatch.reset();
               state.getLogger().info("Sending square up footsteps.");
               computeSquaredUpFootsteps();
               ros2ControllerHelper.publishToController(footstepDataListMessage);
               squareUpFootstepsSent = true;
            }

            if (stopwatch.getElapsedTime() >= 4.0)
            {
               state.getExecutionState().setValue(KickDoorActionExecutionState.STANDING);
               state.setIsExecuting(false);
               stopwatch.reset();
               state.getLogger().info("Going back to standing state.");
               state.setIsExecuting(false);
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

   public void computeTriggerKickCommand(double kickHeight,
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

   public void computeSquaredUpFootsteps()
   {
      MovingReferenceFrame pelvisFrame = syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame();
      FramePose3D kickFootGoalPose = new FramePose3D(pelvisFrame);
      kickFootGoalPose.setY(kickSide.negateIfRightSide(definition.getStanceFootWidth() / 2.0));
      kickFootGoalPose.changeFrame(ReferenceFrame.getWorldFrame());
      kickFootGoalPose.setZ(0.0);
      FootstepDataMessage kickFootStep = HumanoidMessageTools.createFootstepDataMessage(kickSide, kickFootGoalPose);
      footstepDataListMessage.getFootstepDataList().add().set(kickFootStep);

      FramePose3D supportFootGoalPose = new FramePose3D(pelvisFrame);
      supportFootGoalPose.setY(kickSide.negateIfRightSide(-definition.getStanceFootWidth() / 2.0));
      supportFootGoalPose.changeFrame(ReferenceFrame.getWorldFrame());
      supportFootGoalPose.setZ(0.0);
      FootstepDataMessage supportFootStep = HumanoidMessageTools.createFootstepDataMessage(kickSide.getOppositeSide(), supportFootGoalPose);
      footstepDataListMessage.getFootstepDataList().add().set(supportFootStep);
   }
}
