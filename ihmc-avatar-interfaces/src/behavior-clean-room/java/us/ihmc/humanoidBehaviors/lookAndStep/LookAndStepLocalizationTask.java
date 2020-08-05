package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.AngleTools;

import java.util.List;
import java.util.function.Consumer;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.ClosestPointForUI;

public class LookAndStepLocalizationTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   protected RemoteSyncedRobotModel syncedRobot;
   protected Runnable reachedGoalOutput;
   protected Consumer<LookAndStepLocalizationResult> footstepPlanningOutput;

   public static class LookAndStepLocalization extends LookAndStepLocalizationTask
   {
      private SingleThreadSizeOneQueueExecutor executor;
      private final TypedInput<List<? extends Pose3DReadOnly>> bodyPathPlanInput = new TypedInput<>();
      private final Input swingSleepCompleteInput = new Input();

      public void initialize(StatusLogger statusLogger,
                             UIPublisher uiPublisher,
                             LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                             RemoteSyncedRobotModel syncedRobot,
                             Runnable reachedGoalOutput,
                             Consumer<LookAndStepLocalizationResult> footstepPlanningOutput)
      {
         this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
         this.reachedGoalOutput = reachedGoalOutput;
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

      private void snapshotAndRun()
      {
         bodyPathPlan = bodyPathPlanInput.getLatest();
         syncedRobot.update();

         run();
      }
   }

   protected List<? extends Pose3DReadOnly> bodyPathPlan;

   protected void run()
   {
      statusLogger.info("Finding next sub goal for footstep planning...");

      FramePose3D initialPoseBetweenFeet = new FramePose3D();
      initialPoseBetweenFeet.setToZero(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      initialPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = initialPoseBetweenFeet.getZ();

      FramePose3D pelvisPose = new FramePose3D();
      pelvisPose.setToZero(syncedRobot.getReferenceFrames().getPelvisFrame());
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D subGoalPoseBetweenFeet = new FramePose3D();
      subGoalPoseBetweenFeet.setIncludingFrame(pelvisPose);
      subGoalPoseBetweenFeet.setZ(midFeetZ);

      // find closest point along body path plan
      Point3D closestPointAlongPath = new Point3D();
      int closestSegmentIndex = BodyPathPlannerTools.findClosestPointAlongPath(bodyPathPlan, subGoalPoseBetweenFeet.getPosition(), closestPointAlongPath);

      uiPublisher.publishToUI(ClosestPointForUI, new Pose3D(closestPointAlongPath, new Quaternion()));

      Pose3DReadOnly terminalGoal = bodyPathPlan.get(bodyPathPlan.size() - 1);
      double distanceToExactGoal = closestPointAlongPath.distanceXY(terminalGoal.getPosition());
      boolean reachedGoalZone = distanceToExactGoal < lookAndStepBehaviorParameters.getGoalSatisfactionRadius();
      double yawToExactGoal = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(pelvisPose.getYaw(), terminalGoal.getYaw()));
      reachedGoalZone &= yawToExactGoal < lookAndStepBehaviorParameters.getGoalSatisfactionOrientationDelta();

      if (reachedGoalZone)
      {
         statusLogger.warn("Goal reached.");
         reachedGoalOutput.run();
      }
      else
      {
         statusLogger.warn("Remaining travel distance: {} yaw: {}", distanceToExactGoal, yawToExactGoal);
         LookAndStepLocalizationResult result = new LookAndStepLocalizationResult(closestPointAlongPath,
                                                                                  closestSegmentIndex,
                                                                                  subGoalPoseBetweenFeet,
                                                                                  reachedGoalZone,
                                                                                  bodyPathPlan);
         footstepPlanningOutput.accept(result);
      }
   }
}
