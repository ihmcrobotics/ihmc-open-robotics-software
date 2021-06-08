package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import org.lwjgl.Sys;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
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

   private final int SOLUTION_QUALITY_THRESHOLD = 7;
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapAndWiggler snapper;
   private final StepReachabilityData stepReachabilityData;

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
                                          StepReachabilityData stepReachabilityData,
                                          YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.stepReachabilityData = stepReachabilityData;
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

      // Get nearest reachability checkpoint, based on robot side
      candidateFootPose.setReferenceFrame(ReferenceFrame.getWorldFrame());

      // adjust candidate foot position as if it were left footstep
      if (candidateStep.getRobotSide() == RobotSide.RIGHT)
      {
         double stepY = candidateFootPose.getY();
         candidateFootPose.setY(-stepY);

         double stepYaw = candidateFootPose.getYaw();
         double stepPitch = candidateFootPose.getPitch();
         double stepRoll = candidateFootPose.getRoll();
         candidateFootPose.getOrientation().setYawPitchRoll(- stepYaw, stepPitch, stepRoll);
      }

      FramePose3DReadOnly nearestCheckpoint = findNearestCheckpoint(candidateFootPose, reachabilityMap.keySet());

      // Check reachabilityMap to see if candidateFootPose is reachable
      if (!checkpointIsReachable(reachabilityMap, nearestCheckpoint))
         return BipedalFootstepPlannerNodeRejectionReason.REACHABILITY_CHECK;

      return null;
   }

   // Check for closest foot pose in Reachability map keys (looping)
   public FramePose3DReadOnly findNearestCheckpoint(FramePose3D candidateFootPose, Set<? extends FramePose3DReadOnly> mapFootPoses)
   {
      FramePose3DReadOnly nearestCheckpoint = null;
      double smallestDist = 10.0;
      for (FramePose3DReadOnly frame : mapFootPoses)
      {
         double poseDist = frame.getPositionDistance(candidateFootPose);
         double orientationDistance = frame.getOrientationDistance(candidateFootPose);
         if (poseDist+orientationDistance < smallestDist)
         {
            smallestDist = poseDist+orientationDistance;
            nearestCheckpoint = frame;
         }
      }
      return nearestCheckpoint;
   }

   // Check for closest foot pose in Reachability map keys (math)
   // wrote this to possible replace findNearestCheckpoint because checker is sometimes too slow and times out
   // would need to find a way to get queriesPerAxis and offset values to calculate x/y/yawRatio, currently hardcoded
   public FramePose3D updatedFindNearestCheckpoint(FramePose3D candidateFootPose, Set<FramePose3D> MapFootPoses)
   {
      double xRatio = 1/(1.4/52);
      double yRatio = 1/(1.3/52);
      double yawRatio = 1.0/(Math.toRadians(160)/52);

      double newX = candidateFootPose.getX() * xRatio;
      newX = Math.round(newX) / xRatio;

      double newY = candidateFootPose.getY() * yRatio;
      newY = Math.round(newY) / yRatio;

      double newYaw = candidateFootPose.getYaw() * yawRatio;
      newYaw = Math.round(newYaw) / yawRatio;
      FramePose3D nearestCheckpoint = new FramePose3D();
      nearestCheckpoint.setX(newX);
      nearestCheckpoint.setY(newY);
      nearestCheckpoint.getOrientation().setYawPitchRoll(newYaw, 0.0, 0.0);

      return nearestCheckpoint;
   }

   // Check for frame in Reachability map keys
   public boolean checkpointIsReachable(Map<? extends FramePose3DReadOnly, Double> reachabilityMap, FramePose3DReadOnly nearestCheckpoint)
   {
//      System.out.println(nearestCheckpoint);
//      System.out.println(reachabilityMap);
      for (FramePose3DReadOnly frame : reachabilityMap.keySet())
      {
         if (frame.geometricallyEquals(nearestCheckpoint, 0.0001))
         {
//            System.out.println(frame);
            double reachabilityValue = reachabilityMap.get(frame); // TODO Bug: returns null
            return reachabilityValue < SOLUTION_QUALITY_THRESHOLD;
         }
      }
      return false;
   }
}
