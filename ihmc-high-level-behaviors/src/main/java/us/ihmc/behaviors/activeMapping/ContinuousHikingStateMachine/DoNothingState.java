package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import controller_msgs.PauseWalkingMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.TerrainPlanningDebugger;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;

public class DoNothingState implements State
{
   private final ROS2SyncedRobotModel syncedRobotModel;
   private final ContinuousPlanner continuousPlanner;
   private final TerrainPlanningDebugger debugger;

   private final ROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;
   private final SideDependentList<FramePose3D> robotFeet = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   /**
    * This state exists for when the state machine isn't doing anything, if we have gone back to this state after running the state machine, we reset a few
    * things like visuals and some initialization booleans.
    * When we leave this state we re-initialize the continuous planner as this can only mean we are starting things up.
    */
   public DoNothingState(ROS2Node ros2Node,
                         ROS2SyncedRobotModel syncedRobotModel,
                         String simpleRobotName,
                         ContinuousPlanner continuousPlanner,
                         TerrainPlanningDebugger debugger)
   {
      this.syncedRobotModel = syncedRobotModel;
      this.continuousPlanner = continuousPlanner;
      this.debugger = debugger;

      pauseWalkingPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      PauseWalkingMessage message = new PauseWalkingMessage();

      // This state can be entered when we want to stop Continuous Hiking, if that's the cause, pause walking so the robot stops
      if (continuousPlanner.isInitialized())
      {
         message.setPause(true);
         message.setClearRemainingFootstepQueue(true);
         continuousPlanner.setLatestFootstepPlan(null);
         pauseWalkingPublisher.publish(message);
         debugger.resetVisualizationForUIPublisher();
      }

      RigidBodyTransform leftSoleTransform;
      RigidBodyTransform rightSoleTransform;

      // The reference frames are being updated in another thread, to prevent reading when its writing, use the synchronized call
      synchronized (syncedRobotModel)
      {
         leftSoleTransform = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame();
         rightSoleTransform = syncedRobotModel.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();
      }

      robotFeet.get(RobotSide.LEFT).set(leftSoleTransform);
      robotFeet.get(RobotSide.RIGHT).set(rightSoleTransform);
      debugger.publishStartAndGoalForVisualization(robotFeet, robotFeet);

      continuousPlanner.setInitialized(false);
      continuousPlanner.requestMonteCarloPlannerReset();
   }

   @Override
   public void onExit(double timeInState)
   {
      // We are leaving this state, and going to create a footstep plan to use, initialize the continuous planner here
      // This gets initialized here because then it only happens once when we start walking, allowing us to know where we started from
      // THis should not happen at the beginning of the ready to plan state because we may enter that state often when re-planning, but we haven't ever stopped walking
      continuousPlanner.initialize();
      continuousPlanner.setPlanAvailable(false);
   }
}
