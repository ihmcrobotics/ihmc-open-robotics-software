package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FootstepPoseReachabilityChecker
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapperReadOnly snapper;
   private final StepReachabilityData stepReachabilityData;

   private final TransformReferenceFrame stanceFootFrame = new TransformReferenceFrame("stanceFootFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame candidateFootFrame = new TransformReferenceFrame("candidateFootFrame", ReferenceFrame.getWorldFrame());
   private final ZUpFrame stanceFootZUpFrame = new ZUpFrame(stanceFootFrame, "stanceFootZUpFrame");

   /**
    * Robot's stance foot
    */
   private final FramePose3D stanceFootPose = new FramePose3D();
   /**
    * Possible next robot step
    */
   private final FramePose3D candidateFootPose = new FramePose3D();

   private final YoFramePoseUsingYawPitchRoll yoStanceFootPose = new YoFramePoseUsingYawPitchRoll("stance", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoseUsingYawPitchRoll yoCandidateFootPose = new YoFramePoseUsingYawPitchRoll("candidate", stanceFootZUpFrame, registry);

   public FootstepPoseReachabilityChecker(FootstepPlannerParametersReadOnly parameters,
                                          FootstepSnapperReadOnly snapper,
                                          StepReachabilityData stepReachabilityData,
                                          YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.stepReachabilityData = stepReachabilityData;
      parentRegistry.addChild(registry);
   }

   public BipedalFootstepPlannerNodeRejectionReason checkStepValidity(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep)
   {
      RobotSide stepSide = candidateStep.getRobotSide();

      FootstepSnapDataReadOnly candidateStepSnapData = snapper.snapFootstep(candidateStep);
      FootstepSnapDataReadOnly stanceStepSnapData = snapper.snapFootstep(stanceStep);

      candidateFootFrame.setTransformAndUpdate(candidateStepSnapData.getSnappedStepTransform(candidateStep));
      stanceFootFrame.setTransformAndUpdate(stanceStepSnapData.getSnappedStepTransform(stanceStep));
      stanceFootZUpFrame.update();

      candidateFootPose.setToZero(candidateFootFrame);
      candidateFootPose.changeFrame(stanceFootZUpFrame);
      yoCandidateFootPose.set(candidateFootPose);

      stanceFootPose.setToZero(stanceFootFrame);
      stanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      yoStanceFootPose.set(stanceFootPose);

      candidateFootPose.setReferenceFrame(ReferenceFrame.getWorldFrame());

      // Adjust candidate foot position as if it were left footstep
      if (candidateStep.getRobotSide() == RobotSide.RIGHT)
      {
         double stepY = candidateFootPose.getY();
         candidateFootPose.setY(-stepY);

         double stepYaw = candidateFootPose.getYaw();
         double stepPitch = candidateFootPose.getPitch();
         double stepRoll = candidateFootPose.getRoll();
         candidateFootPose.getOrientation().setYawPitchRoll(-stepYaw, stepPitch, stepRoll);
      }

      StepReachabilityLatticePoint nearestReachabilityCheckpoint = new StepReachabilityLatticePoint(candidateFootPose.getX(),
                                                                                                    candidateFootPose.getY(),
                                                                                                    candidateFootPose.getZ(),
                                                                                                    candidateFootPose.getYaw(),
                                                                                                    stepReachabilityData.getXyzSpacing(),
                                                                                                    stepReachabilityData.getYawDivisions(),
                                                                                                    stepReachabilityData.getGridSizeYaw() / stepReachabilityData
                                                                                                          .getYawDivisions());

      // Check reachability map to see if candidate foot position is reachable
      if (stepReachabilityData.getLegReachabilityMap().containsKey(nearestReachabilityCheckpoint))
      {
         if (stepReachabilityData.getLegReachabilityMap().get(nearestReachabilityCheckpoint) < parameters.getSolutionQualityThreshold())
            return null;
      }
      return BipedalFootstepPlannerNodeRejectionReason.REACHABILITY_CHECK;
   }
}
