package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
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
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.string.StringTools;

import java.util.List;
import java.util.function.Consumer;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.ClosestPointForUI;

public class LookAndStepLocalizationTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected RemoteSyncedRobotModel syncedRobot;
   protected Runnable clearAndActivateBodyPathPlanning;
   protected Runnable broadcastReachedGoal;
   protected Consumer<LookAndStepLocalizationResult> footstepPlanningOutput;
   protected Notification finishedWalkingNotification;

   public static class LookAndStepLocalization extends LookAndStepLocalizationTask
   {
      private SingleThreadSizeOneQueueExecutor executor;
      private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
      private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
      private final Input swingSleepCompleteInput = new Input();
      private SideDependentList<PlannedFootstepReadOnly> lastCommandedFootsteps;

      public void initialize(StatusLogger statusLogger,
                             UIPublisher uiPublisher,
                             LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                             RemoteSyncedRobotModel syncedRobot,
                             Notification finishedWalkingNotification,
                             Runnable clearAndActivateBodyPathPlanning,
                             Runnable broadcastReachedGoal,
                             SideDependentList<PlannedFootstepReadOnly> lastCommandedFootsteps,
                             Consumer<LookAndStepLocalizationResult> footstepPlanningOutput)
      {
         this.lastCommandedFootsteps = lastCommandedFootsteps;
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.finishedWalkingNotification = finishedWalkingNotification;
         this.clearAndActivateBodyPathPlanning = clearAndActivateBodyPathPlanning;
         this.broadcastReachedGoal = broadcastReachedGoal;
         this.footstepPlanningOutput = footstepPlanningOutput;
         this.statusLogger = statusLogger;
         this.uiPublisher = uiPublisher;
         this.syncedRobot = syncedRobot;

         executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

         bodyPathPlanInput.addCallback(data -> executor.queueExecution(this::snapshotAndRun));
         swingSleepCompleteInput.addCallback(() -> executor.queueExecution(this::snapshotAndRun));
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
      boolean reachedGoalZone = distanceToExactGoal < lookAndStepBehaviorParameters.getGoalSatisfactionRadius();
      double yawToExactGoal = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(midFeetPose.getYaw(), terminalGoal.getYaw()));
      reachedGoalZone &= yawToExactGoal < lookAndStepBehaviorParameters.getGoalSatisfactionOrientationDelta();

      statusLogger.info(StringTools.format("Eventual pose: {}", StringTools.zUpPoseString(eventualPoseAlongPath)));
      statusLogger.info(StringTools.format("Remaining distanceXY: {} < {} yaw: {} < {}",
                                           FormattingTools.getFormattedDecimal3D(distanceToExactGoal),
                                           lookAndStepBehaviorParameters.getGoalSatisfactionRadius(),
                                           FormattingTools.getFormattedDecimal3D(yawToExactGoal),
                                           lookAndStepBehaviorParameters.getGoalSatisfactionOrientationDelta()));
      if (reachedGoalZone)
      {
         statusLogger.info("Eventual pose reaches goal.");
         clearAndActivateBodyPathPlanning.run();
         ThreadTools.startAsDaemon(this::reachedGoalPublicationThread, "BroadcastReachedGoalWhenDoneWalking");
      }
      else
      {
         LookAndStepLocalizationResult result = new LookAndStepLocalizationResult(closestPointAlongPath,
                                                                                  closestSegmentIndex,
                                                                                  midFeetPose,
                                                                                  bodyPathPlan, eventualStanceFeet);
         footstepPlanningOutput.accept(result);
      }
   }

   /** Here we wait until the robot is finished walking to report reached goal */
   private void reachedGoalPublicationThread()
   {
      statusLogger.info("Waiting for walking to complete...");
      finishedWalkingNotification.poll();
      finishedWalkingNotification.blockingPoll();
      statusLogger.info("Goal reached.");
      broadcastReachedGoal.run();
   }
}
