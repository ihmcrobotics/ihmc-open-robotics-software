package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.HighLevelStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
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
   }

   /** This update is running from the moment this behavior is added to the behavior tree, regardless of whether this behavior is being executed. */
   @Override
   public void update()
   {
      super.update();

      ReferenceFrame doorHandleFrame = referenceFrameLibrary.findFrameByName("doorHandle1");
      // get kick parameters from the definition in here and compute the goal position for the walk action
      // add goal frame to definition or state (probs definition)
   }

   /** Called at the beginning of execution of this behvior. */
   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();


      state.getExecutionState().setValue(KickDoorActionExecutionState.PREPARING_KICK_FOOT);
   }

   /** This is only called while this behavior is being executed. */
   @Override
   public void updateCurrentlyExecuting()
   {
      switch (state.getExecutionState().getValue())
      {
         case STANDING ->
         {
            //Idle until the kick is requested
         }
         case PREPARING_KICK_FOOT ->
         {
            //Move the kicking foot to the desired location
            state.setIsExecuting(true);
         }
         case EXECUTING_KICKING ->
         {
            //Execute the kick
         }
         case KICK_COMPLETED ->
         {
            //Do nothing
            state.setIsExecuting(false);
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

   /**
    * Moves the kicking foot to a new step location based on the output of the {@link us.ihmc.closedSourceControl KickDynamicPlanner}.
    * */
   private void prepareKickFoot(FramePoint2DReadOnly desiredSwingFootStartNominal)
   {
//      // Create the footstep message and send it to the command input manager.
//      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage();
//      ReferenceFrame pelvisFrame = avatarSimulation.getControllerFullRobotModel().getPelvis().getBodyFixedFrame();
//      FramePoint3D location = new FramePoint3D(desiredSwingFootStartNominal);
//      location.setZ(0.0);
//      FrameQuaternion orientation = new FrameQuaternion(pelvisFrame, new Quaternion(0.0, 0.0, 0.0, 1.0));
//      orientation.changeFrame(ReferenceFrame.getWorldFrame());
//      FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(definition.getKickSide(), location, orientation);
//      footsteps.getFootstepDataList().add().set(footstepData);
//      ros2ControllerHelper.publishToController(footsteps);
//      avatarSimulation.getSimulationConstructionSet().simulateNow(2.0);
   }
}
