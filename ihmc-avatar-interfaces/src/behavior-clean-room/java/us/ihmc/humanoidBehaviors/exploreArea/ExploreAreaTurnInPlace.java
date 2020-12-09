package us.ihmc.humanoidBehaviors.exploreArea;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.ParallelNodeBasics;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI.CurrentState;

public class ExploreAreaTurnInPlace extends ParallelNodeBasics
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ExploreAreaBehaviorParameters parameters;
   private final BehaviorHelper helper;
   private final RemoteSyncedRobotModel syncedRobot;
   private final ExploreAreaLatticePlanner exploreAreaLatticePlanner;

   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerLogger footstepPlannerLogger;

   public ExploreAreaTurnInPlace(double expectedTickPeriod,
                                 ExploreAreaBehaviorParameters parameters,
                                 BehaviorHelper helper,
                                 ExploreAreaLatticePlanner exploreAreaLatticePlanner)
   {
      this.parameters = parameters;
      this.helper = helper;
      this.exploreAreaLatticePlanner = exploreAreaLatticePlanner;
      this.footstepPlanner = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());
      this.footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);

      syncedRobot = helper.getOrCreateRobotInterface().newSyncedRobot();
   }

   @Override
   public void doAction()
   {
      helper.publishToUI(CurrentState, ExploreAreaBehavior.ExploreAreaBehaviorState.TurnInPlace);

      // TODO set depending on where the hole is
      double yaw = 1.0;

      turnInPlace(yaw);
      turnInPlace(-yaw);
   }

   public void turnInPlace(double yaw)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setAssumeFlatGround(false);

      syncedRobot.update();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D solePose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide));
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
         request.setStartFootPose(robotSide, solePose);
      }

      FramePose3D goalPose = new FramePose3D(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      goalPose.getOrientation().appendYawRotation(yaw);
      goalPose.getPosition().addX(1e-4);
      request.setGoalFootPoses(footstepPlanner.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);

      request.setPlanBodyPath(false);
      request.setSnapGoalSteps(false);
      FootstepPlannerOutput output = footstepPlanner.handleRequest(request);

      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(output.getFootstepPlan(), -1.0, -1.0);
      TypedNotification<WalkingStatusMessage> walkingCompleted = helper.getOrCreateRobotInterface().requestWalk(footstepDataListMessage);
      walkingCompleted.blockingPoll();

      footstepPlannerLogger.logSession();
   }

}
