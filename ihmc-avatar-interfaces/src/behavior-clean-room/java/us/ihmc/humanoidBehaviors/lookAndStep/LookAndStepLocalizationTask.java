package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidBehaviors.tools.interfaces.UIPublisher;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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
      private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
      private final Input swingSleepCompleteInput = new Input();
      private SideDependentList<PlannedFootstepReadOnly> lastCommandedFootsteps;

      public void initialize(StatusLogger statusLogger,
                             UIPublisher uiPublisher,
                             LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                             RemoteSyncedRobotModel syncedRobot,
                             Runnable reachedGoalOutput,
                             SideDependentList<PlannedFootstepReadOnly> lastCommandedFootsteps,
                             Consumer<LookAndStepLocalizationResult> footstepPlanningOutput)
      {
         this.lastCommandedFootsteps = lastCommandedFootsteps;
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

         // if lastCommandedFootsteps are null we are going to assume the robot is standing still TODO: Check this later
         stanceForChecking = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
         {
            FramePose3D solePose = new FramePose3D();
            if (lastCommandedFootsteps.get(side) == null)
            {
               solePose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
               solePose.changeFrame(ReferenceFrame.getWorldFrame());
               us.ihmc.idl.IDLSequence.Object<Point3D> rawPolygon = side == RobotSide.LEFT ?
                     capturabilityBasedStatus.getLeftFootSupportPolygon2d() : capturabilityBasedStatus.getRightFootSupportPolygon2d();
               ConvexPolygon2D foothold = new ConvexPolygon2D();
               for (Point3D vertex : rawPolygon)
               {
                  foothold.addVertex(vertex);
               }
               stanceForChecking.set(side, new MinimalFootstep(side, solePose, foothold, side.getPascalCaseName() + " Prior Stance"));
            }
            else
            {
               lastCommandedFootsteps.get(side).getFootstepPose(solePose);
               solePose.changeFrame(ReferenceFrame.getWorldFrame());
               stanceForChecking.set(side,
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
   protected SideDependentList<MinimalFootstep> stanceForChecking;
   protected CapturabilityBasedStatus capturabilityBasedStatus;

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
         statusLogger.info("Remaining travel distance: {} yaw: {}", distanceToExactGoal, yawToExactGoal);
         LookAndStepLocalizationResult result = new LookAndStepLocalizationResult(closestPointAlongPath,
                                                                                  closestSegmentIndex,
                                                                                  subGoalPoseBetweenFeet,
                                                                                  bodyPathPlan,
                                                                                  stanceForChecking);
         footstepPlanningOutput.accept(result);
      }
   }
}
