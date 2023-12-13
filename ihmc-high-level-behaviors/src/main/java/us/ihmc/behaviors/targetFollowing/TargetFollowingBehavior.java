package us.ihmc.behaviors.targetFollowing;

import geometry_msgs.PoseStamped;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.ResettingNode;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.Timer;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Follows a person, but keeping a distance. This worked on Atlas and there
 * are good videos of it following Bhavyansh.
 * @deprecated Not supported right now. Being kept for reference or revival.
 */
public class TargetFollowingBehavior extends ResettingNode implements Destroyable
{
   private final BehaviorHelper helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final LookAndStepBehavior lookAndStepBehavior;
   private final TargetFollowingBehaviorParameters targetFollowingParameters;
   private final AtomicReference<PoseStamped> latestSemanticTargetPoseReference = new AtomicReference<>();
   private final Timer lookAndStepGoalSubmissionTimer = new Timer();
   private final FramePose3D targetPoseGroundProjection = new FramePose3D();
   private final FramePose3D approachPose = new FramePose3D();
   private final FramePose3D robotMidFeetUnderPelvisPose = new FramePose3D();
   private final RosTopicPublisher<PoseStamped> targetPosePublisher = null;

   public TargetFollowingBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      syncedRobot = helper.newSyncedRobot();
      LogTools.info("Constructing");
      targetFollowingParameters = new TargetFollowingBehaviorParameters();
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      getChildren().add(lookAndStepBehavior);
//      helper.subscribeViaCallback(TargetFollowingParameters, parameters ->
//      {
//         helper.getOrCreateStatusLogger().info("Accepting new target following parameters");
//         this.targetFollowingParameters.setAllFromStrings(parameters);
//      });
//      helper.getROS1Helper().subscribeToPoseViaCallback(RosTools.SEMANTIC_TARGET_POSE, latestSemanticTargetPoseReference::set);
//      targetPosePublisher = helper.getROS1Helper().publishPose("/ihmc/target_pose_world");
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      PoseStamped latestSemanticTargetPose = latestSemanticTargetPoseReference.getAndSet(null);
      if (latestSemanticTargetPose != null)
      {
         syncedRobot.update();
         robotMidFeetUnderPelvisPose.setToZero(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
         robotMidFeetUnderPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

         targetPoseGroundProjection.setToZero(syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame());
         RosTools.toEuclid(latestSemanticTargetPose.getPose(), targetPoseGroundProjection);
         targetPoseGroundProjection.changeFrame(ReferenceFrame.getWorldFrame());
         targetPoseGroundProjection.getPosition().setZ(robotMidFeetUnderPelvisPose.getZ());
//         helper.publish(TargetPose, new Pose3D(targetPoseGroundProjection));
         PoseStamped ros1Pose = targetPosePublisher.getMessage();
         RosTools.toRos(targetPoseGroundProjection, ros1Pose.getPose());
         ros1Pose.getHeader().setFrameId("world");
//         ros1Pose.getHeader().setStamp(new Time(helper.getROS1Helper().getROS1Node().getCurrentTime().totalNsecs()));
         targetPosePublisher.publish(ros1Pose);

         approachPose.set(targetPoseGroundProjection);
         Vector3D fromTarget = new Vector3D();
         fromTarget.sub(robotMidFeetUnderPelvisPose.getPosition(), targetPoseGroundProjection.getPosition());
         fromTarget.normalize();
         fromTarget.scale(targetFollowingParameters.getMinimumDistanceToKeepFromTarget());
         approachPose.getPosition().add(fromTarget);
         Vector3D toTarget = new Vector3D();
         toTarget.sub(targetPoseGroundProjection.getPosition(), robotMidFeetUnderPelvisPose.getPosition());
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X, toTarget, approachPose.getOrientation());
//         helper.publish(TargetApproachPose, new Pose3D(approachPose));

         if (!lookAndStepGoalSubmissionTimer.isRunning(targetFollowingParameters.getLookAndStepGoalUpdatePeriod()))
         {
            lookAndStepBehavior.acceptGoal(approachPose);
            lookAndStepGoalSubmissionTimer.reset();
         }
      }

      return lookAndStepBehavior.tickAndGetStatus();
   }

   @Override
   public void reset()
   {

   }

   public String getName()
   {
      return "Target Following";
   }

   @Override
   public void destroy()
   {
      lookAndStepBehavior.destroy();
   }
}


