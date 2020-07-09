package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.function.UnaryOperator;

public class GoodFootstepPositionChecker
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapAndWiggler snapper;
   private final FootstepPlannerEdgeData edgeData;

   private final TransformReferenceFrame startOfSwingFrame = new TransformReferenceFrame("startOfSwingFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame stanceFootFrame = new TransformReferenceFrame("stanceFootFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame candidateFootFrame = new TransformReferenceFrame("candidateFootFrame", ReferenceFrame.getWorldFrame());
   private final ZUpFrame startOfSwingZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), startOfSwingFrame, "startOfSwingZUpFrame");
   private final ZUpFrame stanceFootZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), stanceFootFrame, "stanceFootZUpFrame");
   private final FramePose3D stanceFootPose = new FramePose3D();
   private final FramePose3D candidateFootPose = new FramePose3D();

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason;
   private UnaryOperator<FootstepNode> parentNodeSupplier;

   // Variables to log
   private double stepWidth;
   private double stepLength;
   private double stepHeight;
   private double stepReachXY;

   public GoodFootstepPositionChecker(FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapAndWiggler snapper, FootstepPlannerEdgeData edgeData)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.edgeData = edgeData;
   }

   public void setParentNodeSupplier(UnaryOperator<FootstepNode> parentNodeSupplier)
   {
      this.parentNodeSupplier = parentNodeSupplier;
   }

   public boolean isNodeValid(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      RobotSide stepSide = candidateNode.getRobotSide();

      FootstepNodeSnapData candidateNodeSnapData = snapper.snapFootstepNode(candidateNode);
      FootstepNodeSnapData stanceNodeSnapData = snapper.snapFootstepNode(stanceNode);

      candidateFootFrame.setTransformAndUpdate(candidateNodeSnapData.getSnappedNodeTransform(candidateNode));
      stanceFootFrame.setTransformAndUpdate(stanceNodeSnapData.getSnappedNodeTransform(stanceNode));
      stanceFootZUpFrame.update();

      candidateFootPose.setToZero(candidateFootFrame);
      candidateFootPose.changeFrame(stanceFootZUpFrame);

      stanceFootPose.setToZero(stanceFootFrame);
      stanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      stepLength = candidateFootPose.getX();
      stepWidth = stepSide.negateIfRightSide(candidateFootPose.getY());
      stepReachXY = EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(candidateFootPose.getX()), Math.abs(stepWidth - parameters.getIdealFootstepWidth()));
      stepHeight = candidateFootPose.getZ();
      double maximumStepZ = stepSide == RobotSide.LEFT ? parameters.getMaximumLeftStepZ() : parameters.getMaximumRightStepZ();

      if (stepWidth < parameters.getMinimumStepWidth())
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH;
         return false;
      }
      else if (stepWidth > parameters.getMaximumStepWidth())
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE;
         return false;
      }
      else if (stepLength < parameters.getMinimumStepLength())
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH;
         return false;
      }
      else if (Math.abs(stepHeight) > maximumStepZ)
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW;
         return false;
      }

      double alphaPitchedBack = Math.max(0.0, - stanceFootPose.getPitch() / parameters.getMinimumSurfaceInclineRadians());
      double minZFromPitchContraint = InterpolationTools.linearInterpolate(Math.abs(maximumStepZ), Math.abs(parameters.getMinimumStepZWhenFullyPitched()), alphaPitchedBack);
      double maxXFromPitchContraint = InterpolationTools.linearInterpolate(Math.abs(parameters.getMaximumStepReach()), parameters.getMaximumStepXWhenFullyPitched(), alphaPitchedBack);
      double stepDownFraction = - stepHeight / minZFromPitchContraint;
      double stepForwardFraction = stepLength / maxXFromPitchContraint;

      boolean stepIsPitchedBack = alphaPitchedBack > 0.0;
      boolean stepTooLow = stepLength > 0.0 && stepDownFraction > 1.0;
      boolean stepTooForward = stepHeight < 0.0 && stepForwardFraction > 1.0;

      if (stepIsPitchedBack && (stepTooLow || stepTooForward))
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_LOW_AND_FORWARD_WHEN_PITCHED;
         return false;
      }

      double maxReach = parameters.getMaximumStepReach();
      if (stepHeight < -Math.abs(parameters.getMaximumStepZWhenForwardAndDown()))
      {
         if (stepLength > parameters.getMaximumStepXWhenForwardAndDown())
         {
            rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN;
            return false;
         }

         if (stepWidth > parameters.getMaximumStepYWhenForwardAndDown())
         {
            rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE_AND_DOWN;
            return false;
         }

         maxReach = EuclidCoreTools.norm(parameters.getMaximumStepXWhenForwardAndDown(), parameters.getMaximumStepYWhenForwardAndDown() - parameters.getIdealFootstepWidth());
      }

      if (stepReachXY > parameters.getMaximumStepReach())
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR;
         return false;
      }

      if (stepHeight > parameters.getMaximumStepZWhenSteppingUp())
      {
         if (stepReachXY > parameters.getMaximumStepReachWhenSteppingUp())
         {
            rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_AND_HIGH;
            return false;
         }
         if (stepWidth > parameters.getMaximumStepWidthWhenSteppingUp())
         {
            rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE_AND_HIGH;
            return false;
         }

         maxReach = parameters.getMaximumStepReachWhenSteppingUp();
      }

      double stepReach3D = EuclidCoreTools.norm(stepReachXY, stepHeight);
      double maxInterpolationFactor = Math.max(stepReach3D / maxReach, Math.abs(stepHeight / maximumStepZ));
      maxInterpolationFactor = Math.min(maxInterpolationFactor, 1.0);
      double maxYaw = InterpolationTools.linearInterpolate(parameters.getMaximumStepYaw(), (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMaximumStepYaw(),
                                                           maxInterpolationFactor);
      double minYaw = InterpolationTools.linearInterpolate(parameters.getMinimumStepYaw(), (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMinimumStepYaw(),
                                                           maxInterpolationFactor);
      double yawDelta = AngleTools.computeAngleDifferenceMinusPiToPi(candidateNode.getYaw(), stanceNode.getYaw());
      if (!MathTools.intervalContains(stepSide.negateIfRightSide(yawDelta), minYaw, maxYaw))
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH;
         return false;
      }

      // Check reach from start of swing
      FootstepNode grandParentNode;
      FootstepNodeSnapData grandparentNodeSnapData;
      double alphaSoS = parameters.getTranslationScaleFromGrandparentNode();
      if (alphaSoS > 0.0 && parentNodeSupplier != null && (grandParentNode = parentNodeSupplier.apply(stanceNode)) != null
          && (grandparentNodeSnapData = snapper.snapFootstepNode(grandParentNode)) != null)
      {
         startOfSwingFrame.setTransformAndUpdate(grandparentNodeSnapData.getSnappedNodeTransform(grandParentNode));
         startOfSwingZUpFrame.update();
         candidateFootPose.changeFrame(startOfSwingZUpFrame);
         double swingHeight = candidateFootPose.getZ();
         double swingReach = EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(candidateFootPose.getX()), Math.abs(candidateFootPose.getY()));

         if (swingHeight > parameters.getMaximumStepZWhenSteppingUp())
         {
            if (swingReach > alphaSoS * parameters.getMaximumStepReachWhenSteppingUp())
            {
               rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_AND_HIGH;
               return false;
            }
         }
      }

      return true;
   }

   void logVariables()
   {
      if (edgeData != null)
      {
         edgeData.setStepWidth(stepWidth);
         edgeData.setStepLength(stepLength);
         edgeData.setStepHeight(stepHeight);
         edgeData.setStepReach(stepReachXY);
      }
   }

   void clearLoggedVariables()
   {
      stepWidth = Double.NaN;
      stepLength = Double.NaN;
      stepHeight = Double.NaN;
      stepReachXY = Double.NaN;
   }

   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason;
   }
}
