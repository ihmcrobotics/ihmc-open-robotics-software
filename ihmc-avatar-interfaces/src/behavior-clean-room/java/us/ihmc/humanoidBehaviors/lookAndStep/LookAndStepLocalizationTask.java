package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBodyPathPlanningTask.LookAndStepBodyPathPlanning;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.humanoidBehaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.string.StringTools;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.ClosestPointForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.REACHED_GOAL;

public class LookAndStepLocalizationTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected Consumer<ROS2Topic<Empty>> ros2EmptyPublisher;
   protected LookAndStepBodyPathPlanning bodyPathPlanning;
   protected LookAndStepSteppingTask.LookAndStepStepping stepping;
   protected AtomicBoolean isBeingReset;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   protected RemoteSyncedRobotModel syncedRobot;
   protected Consumer<LookAndStepBodyPathLocalizationResult> bodyPathLocalizationOutput;
   protected Notification finishedWalkingNotification;
   protected BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference;
   protected ControllerStatusTracker controllerStatusTracker;

   public static class LookAndStepBodyPathLocalization extends LookAndStepLocalizationTask
   {
      private SingleThreadSizeOneQueueExecutor executor;
      private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
      private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
      private final Input swingSleepCompleteInput = new Input();
      private SideDependentList<PlannedFootstepReadOnly> lastCommandedFootsteps;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         lastCommandedFootsteps = lookAndStep.lastCommandedFootsteps;
         lookAndStepParameters = lookAndStep.lookAndStepParameters;
         finishedWalkingNotification = lookAndStep.helper.createWalkingCompletedNotification();
         ros2EmptyPublisher = lookAndStep.helper::publishROS2;
         bodyPathPlanning = lookAndStep.bodyPathPlanning;
         behaviorStateReference = lookAndStep.behaviorStateReference;
         bodyPathLocalizationOutput = lookAndStep.footstepPlanning::acceptLocalizationResult;
         statusLogger = lookAndStep.statusLogger;
         uiPublisher = lookAndStep.helper::publishToUI;
         syncedRobot = lookAndStep.robotInterface.newSyncedRobot();
         isBeingReset = lookAndStep.isBeingReset;
         stepping = lookAndStep.stepping;
         controllerStatusTracker = lookAndStep.controllerStatusTracker;

         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

         bodyPathPlanInput.addCallback(data -> executor.submitTask(this::snapshotAndRun));
         swingSleepCompleteInput.addCallback(() -> executor.submitTask(this::snapshotAndRun));
      }

      public void acceptBodyPathPlan(List<? extends Pose3DReadOnly> bodyPathPlan)
      {
         bodyPathPlanInput.set(bodyPathPlan);
      }

      public void acceptSwingSleepComplete()
      {
         swingSleepCompleteInput.set();
      }

      public void acceptCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
      {
         capturabilityBasedStatusInput.set(capturabilityBasedStatus);
      }

      public void reset()
      {
         executor.interruptAndReset();
      }

      private void snapshotAndRun()
      {
         bodyPathPlan = bodyPathPlanInput.getLatest();
         capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
         syncedRobot.update();

         eventualStanceFeet = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
         {
            FramePose3D solePose = new FramePose3D();
            if (lastCommandedFootsteps.get(side) == null) // in the case we are just starting to walk and haven't sent a step for this foot yet
            {
               solePose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
               solePose.changeFrame(ReferenceFrame.getWorldFrame());
               us.ihmc.idl.IDLSequence.Object<Point3D> rawPolygon = side == RobotSide.LEFT ?
                     capturabilityBasedStatus.getLeftFootSupportPolygon3d() : capturabilityBasedStatus.getRightFootSupportPolygon3d();
               ConvexPolygon2D foothold = new ConvexPolygon2D();
               for (Point3D vertex : rawPolygon)
               {
                  foothold.addVertex(vertex);
               }
               eventualStanceFeet.set(side, new MinimalFootstep(side, solePose, foothold, side.getPascalCaseName() + " Prior Stance"));
            }
            else
            {
               lastCommandedFootsteps.get(side).getFootstepPose(solePose);
               solePose.changeFrame(ReferenceFrame.getWorldFrame());
               eventualStanceFeet.set(side,
                                      new MinimalFootstep(side,
                                                         solePose,
                                                         lastCommandedFootsteps.get(side).getFoothold(),
                                                         side.getPascalCaseName() + " Commanded Stance"));
            }
         }

         run();
      }
   }

   protected List<? extends Pose3DReadOnly> bodyPathPlan;
   protected SideDependentList<MinimalFootstep> eventualStanceFeet;
   protected CapturabilityBasedStatus capturabilityBasedStatus;

   protected void run()
   {
      statusLogger.info("Localizing eventual pose to body path...");

      Pose3D midFeetPose = new Pose3D(eventualStanceFeet.get(RobotSide.LEFT).getSolePoseInWorld());
      midFeetPose.interpolate(eventualStanceFeet.get(RobotSide.RIGHT).getSolePoseInWorld(), 0.5);

      Vector3D midFeetZNormal = new Vector3D(Axis3D.Z);
      midFeetPose.getOrientation().transform(midFeetZNormal);
      Quaternion toZUp = new Quaternion();
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(midFeetZNormal, Axis3D.Z, toZUp);
      midFeetPose.appendRotation(toZUp);

      // find closest point along body path plan
      Point3D closestPointAlongPath = new Point3D();
      int closestSegmentIndex = BodyPathPlannerTools.findClosestPointAlongPath(bodyPathPlan, midFeetPose.getPosition(), closestPointAlongPath);

      Pose3D eventualPoseAlongPath = new Pose3D(midFeetPose);
      eventualPoseAlongPath.getPosition().set(closestPointAlongPath);
      uiPublisher.publishToUI(ClosestPointForUI, eventualPoseAlongPath);

      Pose3DReadOnly terminalGoal = bodyPathPlan.get(bodyPathPlan.size() - 1);
      double distanceToExactGoal = closestPointAlongPath.distanceXY(terminalGoal.getPosition());
      boolean reachedGoalZone = distanceToExactGoal < lookAndStepParameters.getGoalSatisfactionRadius();
      double yawToExactGoal = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(midFeetPose.getYaw(), terminalGoal.getYaw()));
      reachedGoalZone &= yawToExactGoal < lookAndStepParameters.getGoalSatisfactionOrientationDelta();

      statusLogger.info(StringTools.format("Eventual pose: {}", StringTools.zUpPoseString(eventualPoseAlongPath)));
      statusLogger.info(StringTools.format("Remaining distanceXY: {} < {} yaw: {} < {}",
                                           FormattingTools.getFormattedDecimal3D(distanceToExactGoal),
                                           lookAndStepParameters.getGoalSatisfactionRadius(),
                                           FormattingTools.getFormattedDecimal3D(yawToExactGoal),
                                           lookAndStepParameters.getGoalSatisfactionOrientationDelta()));
      if (reachedGoalZone)
      {
         statusLogger.info("Eventual pose reaches goal.");
         if ((!isBeingReset.get()))
         {
            ros2EmptyPublisher.accept(SLAMModuleAPI.CLEAR);
            bodyPathPlanning.acceptGoal(null);
            behaviorStateReference.set(LookAndStepBehavior.State.BODY_PATH_PLANNING);
         }
         ThreadTools.startAsDaemon(this::reachedGoalPublicationThread, "BroadcastReachedGoalWhenDoneWalking");
      }
      else
      {
         LookAndStepBodyPathLocalizationResult result = new LookAndStepBodyPathLocalizationResult(closestPointAlongPath,
                                                                                                  closestSegmentIndex,
                                                                                                  midFeetPose,
                                                                                                  bodyPathPlan,
                                                                                                  eventualStanceFeet);
         bodyPathLocalizationOutput.accept(result);
      }
   }

   /** Here we wait until the robot is finished walking to report reached goal */
   private void reachedGoalPublicationThread()
   {
      statusLogger.info("Waiting for walking to complete...");
      finishedWalkingNotification.poll();
      if (controllerStatusTracker.isWalking())
      {
         finishedWalkingNotification.blockingPoll();
      }
      stepping.reset();
      statusLogger.info("Goal reached.");
      ros2EmptyPublisher.accept(REACHED_GOAL);
   }
}
