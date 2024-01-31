package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import std_msgs.msg.dds.Empty;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.lookAndStep.LookAndStepBodyPathPlanningTask.LookAndStepBodyPathPlanning;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.interfaces.UIPublisher;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepLocalizationTask
{
   protected StatusLogger statusLogger;
   protected BehaviorHelper helper;
   protected UIPublisher uiPublisher;
   protected Consumer<ROS2Topic<Empty>> ros2EmptyPublisher;
   protected LookAndStepBodyPathPlanning bodyPathPlanning;
   protected LookAndStepSteppingTask.LookAndStepStepping stepping;
   protected AtomicBoolean isBeingReset;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   protected ROS2SyncedRobotModel syncedRobot;
   protected Consumer<LookAndStepBodyPathLocalizationResult> bodyPathLocalizationOutput;
   protected Notification finishedWalkingNotification;
   protected BehaviorStateReference<LookAndStepBehavior.State> behaviorStateReference;
   protected ControllerStatusTracker controllerStatusTracker;
   protected boolean newGoalSubmitted = false;
   protected boolean didFootstepPlanningOnceToEnsureSomeProgress = false;

   public static class LookAndStepBodyPathLocalization extends LookAndStepLocalizationTask
   {
      private ResettableExceptionHandlingExecutorService executor;
      private ROS2StoredPropertySet<LookAndStepBehaviorParametersBasics> ros2LookAndStepParameters;
      private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
      private final Input swingSleepCompleteInput = new Input();
      private LookAndStepImminentStanceTracker imminentStanceTracker;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         imminentStanceTracker = lookAndStep.imminentStanceTracker;
         ros2LookAndStepParameters = lookAndStep.ros2LookAndStepParameters;
         lookAndStepParameters = ros2LookAndStepParameters.getStoredPropertySet();
         finishedWalkingNotification = lookAndStep.helper.subscribeToWalkingCompletedViaNotification();
         helper = lookAndStep.helper;
         ros2EmptyPublisher = lookAndStep.helper::publish;
         bodyPathPlanning = lookAndStep.bodyPathPlanning;
         behaviorStateReference = lookAndStep.behaviorStateReference;
         bodyPathLocalizationOutput = lookAndStep.footstepPlanning::acceptLocalizationResult;
         statusLogger = lookAndStep.statusLogger;
         uiPublisher = lookAndStep.helper::publish;
         syncedRobot = lookAndStep.robotInterface.newSyncedRobot();
         isBeingReset = lookAndStep.isBeingReset;
         stepping = lookAndStep.stepping;
         controllerStatusTracker = lookAndStep.controllerStatusTracker;

         executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

         bodyPathPlanInput.addCallback(data -> executor.clearQueueAndExecute(this::snapshotAndRun));
         swingSleepCompleteInput.addCallback(() -> executor.clearQueueAndExecute(this::snapshotAndRun));
      }

      public void acceptBodyPathPlan(List<? extends Pose3DReadOnly> bodyPathPlan)
      {
         newGoalSubmitted = false;
         bodyPathPlanInput.set(bodyPathPlan);
      }

      public void acceptSwingSleepComplete()
      {
         swingSleepCompleteInput.set();
      }

      public void acceptNewGoalSubmitted()
      {
         newGoalSubmitted = true;
      }

      public void reset()
      {
         newGoalSubmitted = false;
         didFootstepPlanningOnceToEnsureSomeProgress = false;
         executor.interruptAndReset();
      }

      private void snapshotAndRun()
      {
         ros2LookAndStepParameters.update();
         bodyPathPlan = bodyPathPlanInput.getLatest();
         syncedRobot.update();
         imminentStanceFeet = imminentStanceTracker.calculateImminentStancePoses();

         run();
      }
   }

   protected List<? extends Pose3DReadOnly> bodyPathPlan;
   protected SideDependentList<MinimalFootstep> imminentStanceFeet;
   protected CapturabilityBasedStatus capturabilityBasedStatus;

   protected void run()
   {
      statusLogger.info("Localizing imminent pose to body path...");

      Pose3D imminentMidFeetPose = new Pose3D(imminentStanceFeet.get(RobotSide.LEFT).getSolePoseInWorld());
      imminentMidFeetPose.interpolate(imminentStanceFeet.get(RobotSide.RIGHT).getSolePoseInWorld(), 0.5);

      Vector3D midFeetZNormal = new Vector3D(Axis3D.Z);
      imminentMidFeetPose.getOrientation().transform(midFeetZNormal);
      Quaternion toZUp = new Quaternion();
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(midFeetZNormal, Axis3D.Z, toZUp);
      imminentMidFeetPose.appendRotation(toZUp);

      // find closest point along body path plan
      Pose3D closestPoseAlongPath = new Pose3D();
      int closestSegmentIndex = BodyPathPlannerTools.findClosestPoseAlongPath(bodyPathPlan, imminentMidFeetPose.getPosition(), closestPoseAlongPath);

      helper.publish(CLOSEST_POINT_FOR_UI, closestPoseAlongPath);

      Pose3DReadOnly terminalGoal = bodyPathPlan.get(bodyPathPlan.size() - 1);
      double distanceToExactGoal = imminentMidFeetPose.getPosition().distanceXY(terminalGoal.getPosition());
      boolean reachedGoalZone = distanceToExactGoal < lookAndStepParameters.getGoalSatisfactionRadius();
      double yawToExactGoal = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(imminentMidFeetPose.getYaw(), terminalGoal.getYaw()));
      reachedGoalZone &= yawToExactGoal < lookAndStepParameters.getGoalSatisfactionOrientationDelta();

      statusLogger.info(StringTools.format("Remaining distanceXY: {} < {} yaw: {} < {}",
                                           FormattingTools.getFormattedDecimal3D(distanceToExactGoal),
                                           lookAndStepParameters.getGoalSatisfactionRadius(),
                                           FormattingTools.getFormattedDecimal3D(yawToExactGoal),
                                           lookAndStepParameters.getGoalSatisfactionOrientationDelta()));
      boolean newGoalWasSubmitted = newGoalSubmitted;
      newGoalSubmitted = false;
      if (reachedGoalZone)
      {
         statusLogger.info("Imminent pose reaches goal.");
         if ((!isBeingReset.get()))
         {
            ros2EmptyPublisher.accept(SLAMModuleAPI.CLEAR);
            bodyPathPlanning.acceptGoal(null);
            behaviorStateReference.set(LookAndStepBehavior.State.BODY_PATH_PLANNING);
         }
         ThreadTools.startAsDaemon(this::reachedGoalPublicationThread, "BroadcastReachedGoalWhenDoneWalking");
      }
//      else if (newGoalWasSubmitted && didFootstepPlanningOnceToEnsureSomeProgress)
      else if (newGoalWasSubmitted)
      {
         didFootstepPlanningOnceToEnsureSomeProgress = false;
         statusLogger.info("Planning to new goal.");
         if ((!isBeingReset.get()))
         {
            behaviorStateReference.set(LookAndStepBehavior.State.BODY_PATH_PLANNING);
            bodyPathPlanning.run();
         }
      }
      else
      {
         didFootstepPlanningOnceToEnsureSomeProgress = true;
         LookAndStepBodyPathLocalizationResult result = new LookAndStepBodyPathLocalizationResult(closestPoseAlongPath,
                                                                                                  closestSegmentIndex,
                                                                                                  imminentMidFeetPose,
                                                                                                  bodyPathPlan);
         behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
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
