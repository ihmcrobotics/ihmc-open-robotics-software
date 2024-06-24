package us.ihmc.behaviors.activeMapping.ContinuousHikingStatesAndTransitions;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2PublisherBasics;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class DoNothingState implements State
{
   private final HumanoidReferenceFrames referenceFrames;
   private final ContinuousPlanner continuousPlanner;
   private final TerrainPlanningDebugger debugger;

   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

   public DoNothingState(ROS2Helper ros2Helper,
                         String simpleRobotName,
                         HumanoidReferenceFrames referenceFrames,
                         ContinuousPlanner continuousPlanner,
                         TerrainPlanningDebugger debugger)
   {
      this.referenceFrames = referenceFrames;
      this.continuousPlanner = continuousPlanner;
      this.debugger = debugger;

      pauseWalkingPublisher = ros2Helper.getROS2NodeInterface().createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));
   }


   @Override
   public void onEntry()
   {
      LogTools.warn("Entering [DO_NOTHING] state");
   }

   @Override
   public void doAction(double timeInState)
   {
      PauseWalkingMessage message = new PauseWalkingMessage();

      if (continuousPlanner.isInitialized())
      {
         message.setPause(true);
         message.setClearRemainingFootstepQueue(true);
         pauseWalkingPublisher.publish(message);
      }

      RobotSide closerSide = continuousPlanner.getCloserSideToGoal();
      FramePose3D closerToGoalFootPose = new FramePose3D(referenceFrames.getSoleFrame(closerSide));
      FramePose3D fartherToGoalFootPose = new FramePose3D(referenceFrames.getSoleFrame(closerSide.getOppositeSide()));
      closerToGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      fartherToGoalFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      if (continuousPlanner.updateImminentStance(fartherToGoalFootPose, closerToGoalFootPose, closerSide))
      {
         debugger.publishStartAndGoalForVisualization(continuousPlanner.getStartingStancePose(), continuousPlanner.getGoalStancePose());
      }

      continuousPlanner.setInitialized(false);
      continuousPlanner.requestMonteCarloPlannerReset();
   }

   @Override
   public void onExit(double timeInState)
   {
   }
}
