package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Map;
import java.util.Set;

public class FootstepPoseReachabilityChecker
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapAndWiggler snapper;
   private final Map<FramePose3D, Boolean> reachabilityMap;

   private final TransformReferenceFrame stanceFootFrame = new TransformReferenceFrame("stanceFootFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame candidateFootFrame = new TransformReferenceFrame("candidateFootFrame", ReferenceFrame.getWorldFrame());
   private final ZUpFrame stanceFootZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), stanceFootFrame, "stanceFootZUpFrame");

   /** Robot's stance foot */
   private final FramePose3D stanceFootPose = new FramePose3D();
   /** Possible next robot step */
   private final FramePose3D candidateFootPose = new FramePose3D();

   private final YoFramePoseUsingYawPitchRoll yoStanceFootPose = new YoFramePoseUsingYawPitchRoll("stance", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoseUsingYawPitchRoll yoCandidateFootPose = new YoFramePoseUsingYawPitchRoll("candidate", stanceFootZUpFrame, registry);

   public FootstepPoseReachabilityChecker(FootstepPlannerParametersReadOnly parameters,
                                          FootstepSnapAndWiggler snapper,
                                          Map<FramePose3D, Boolean> reachabilityMap,
                                          YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.reachabilityMap = reachabilityMap;
      parentRegistry.addChild(registry);
   }

   public BipedalFootstepPlannerNodeRejectionReason checkStepValidity(DiscreteFootstep candidateStep,
                                                                      DiscreteFootstep stanceStep)
   {
      RobotSide stepSide = candidateStep.getRobotSide();

      FootstepSnapData candidateStepSnapData = snapper.snapFootstep(candidateStep);
      FootstepSnapData stanceStepSnapData = snapper.snapFootstep(stanceStep);

      candidateFootFrame.setTransformAndUpdate(candidateStepSnapData.getSnappedStepTransform(candidateStep));
      stanceFootFrame.setTransformAndUpdate(stanceStepSnapData.getSnappedStepTransform(stanceStep));
      stanceFootZUpFrame.update();

      candidateFootPose.setToZero(candidateFootFrame);
      candidateFootPose.changeFrame(stanceFootZUpFrame);
      yoCandidateFootPose.set(candidateFootPose);

      stanceFootPose.setToZero(stanceFootFrame);
      stanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      yoStanceFootPose.set(stanceFootPose);

      // Translate candidateStep to nearest reachability checkpoint
      FramePose3D nearestCheckpoint = findNearestCheckpoint(candidateFootPose, reachabilityMap.keySet());

      // Check reachabilityMap to see if candidateFootPose is reachable
      if (!reachabilityMap.get(nearestCheckpoint)) return BipedalFootstepPlannerNodeRejectionReason.REACHABILITY_CHECK;

      return null;
   }

   // Check for closest foot pose in Reachability map keys
   public FramePose3D findNearestCheckpoint(FramePose3D candidateFootPose, Set<FramePose3D> MapFootPoses)
   {
      FramePose3D nearestCheckpoint = new FramePose3D();
      double smallestDist = 10.0;
      for (FramePose3D pose : MapFootPoses)
      {
         double poseDist = pose.getPositionDistance(candidateFootPose);
         double orientationDistance = pose.getOrientationDistance(candidateFootPose);
         if (poseDist+orientationDistance < smallestDist)
         {
            smallestDist = poseDist+orientationDistance;
            nearestCheckpoint = pose;
         }
      }
      return nearestCheckpoint;
   }
}
