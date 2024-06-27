package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ContinuousPlannerTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2PublisherBasics;

public class DoNothingState implements State
{
   private final HumanoidReferenceFrames referenceFrames;
   private final ContinuousPlanner continuousPlanner;

   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

   public DoNothingState(ROS2Helper ros2Helper,
                         String simpleRobotName,
                         HumanoidReferenceFrames referenceFrames,
                         ContinuousPlanner continuousPlanner)
   {
      this.referenceFrames = referenceFrames;
      this.continuousPlanner = continuousPlanner;

      pauseWalkingPublisher = ros2Helper.getROS2NodeInterface().createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));
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
      }

      RobotSide closerSide = ContinuousPlannerTools.getCloserSideToGoal(continuousPlanner.getStartStancePose(), continuousPlanner.getGoalStancePose());
      FramePose3D closerToGoalFootPose = new FramePose3D(referenceFrames.getSoleFrame(closerSide));
      FramePose3D fartherToGoalFootPose = new FramePose3D(referenceFrames.getSoleFrame(closerSide.getOppositeSide()));
      closerToGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      fartherToGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      continuousPlanner.setInitialized(false);
      continuousPlanner.requestMonteCarloPlannerReset();
   }

   @Override
   public void onExit(double timeInState)
   {
      // We are leaving this state, and going to create a footstep plan to use, initialize the continuous planner here
      // This gets initialized here because then it only happens once when we start walking, allowing us to know where we started from
      continuousPlanner.initialize();
      continuousPlanner.setPlanAvailable(false);
   }
}
