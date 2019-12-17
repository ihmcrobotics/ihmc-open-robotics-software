package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class GoodFootstepPositionChecker implements SnapBasedCheckerComponent
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TransformReferenceFrame parentSoleFrame = new TransformReferenceFrame("parentSole", ReferenceFrame.getWorldFrame());
   private final ZUpFrame parentSoleZupFrame = new ZUpFrame(worldFrame, parentSoleFrame, "parentSoleZupFrame");
   private final TransformReferenceFrame nodeSoleFrame = new TransformReferenceFrame("nodeSole", ReferenceFrame.getWorldFrame());
   private final FramePoint3D solePositionInParentZUpFrame = new FramePoint3D(parentSoleZupFrame);

   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapper snapper;

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason;
   private DirectedGraph<FootstepNode> graph;

   public GoodFootstepPositionChecker(FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public void setFootstepGraph(DirectedGraph<FootstepNode> graph)
   {
      this.graph = graph;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {

   }

   @Override
   public boolean isNodeValid(FootstepNode nodeToCheck, FootstepNode previousNode)
   {
      if (previousNode == null)
         return true;

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(nodeToCheck);
      FootstepNodeSnapData previousSnapData = snapper.snapFootstepNode(previousNode);

      RigidBodyTransformReadOnly snappedSoleTransform = snapData.getOrComputeSnappedNodeTransform(nodeToCheck);
      RigidBodyTransformReadOnly previousSnappedSoleTransform = previousSnapData.getOrComputeSnappedNodeTransform(previousNode);

      parentSoleFrame.setTransformAndUpdate(previousSnappedSoleTransform);
      parentSoleZupFrame.update();

      nodeSoleFrame.setTransformAndUpdate(snappedSoleTransform);

      solePositionInParentZUpFrame.setToZero(nodeSoleFrame);
      solePositionInParentZUpFrame.changeFrame(parentSoleZupFrame);

      RobotSide robotSide = nodeToCheck.getRobotSide();
      double sidedWidth = robotSide.negateIfRightSide(solePositionInParentZUpFrame.getY());
      if (sidedWidth <  parameters.getMinimumStepWidth())
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH;
         return false;
      }

      if (sidedWidth > parameters.getMaximumStepWidth())
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE;
         return false;
      }

      double minimumStepLength = parameters.getMinimumStepLength();
      if (solePositionInParentZUpFrame.getX() < minimumStepLength)
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH;
         return false;
      }

      if (Math.abs(solePositionInParentZUpFrame.getZ()) > parameters.getMaximumStepZ())
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW;
         return false;
      }

      double alpha = Math.max(0.0, -previousSnappedSoleTransform.getRotation().getPitch() / parameters.getMinimumSurfaceInclineRadians());
      double minZ = InterpolationTools.linearInterpolate(Math.abs(parameters.getMaximumStepZ()), Math.abs(parameters.getMinimumStepZWhenFullyPitched()), alpha);
      double minX = InterpolationTools.linearInterpolate(Math.abs(parameters.getMaximumStepReach()), parameters.getMaximumStepXWhenFullyPitched(), alpha);
      double stepDownFraction = -solePositionInParentZUpFrame.getZ() / minZ;
      double stepForwardFraction = solePositionInParentZUpFrame.getX() / minX;

      // TODO eliminate the 1.5, and look at the actual max step z and max step reach to ensure those are valid if there's not any pitching
      if (stepDownFraction > 1.0 || stepForwardFraction > 1.0 || (stepDownFraction + stepForwardFraction > 1.5))
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_LOW_AND_FORWARD_WHEN_PITCHED;
         return false;
      }

      double maxReach = parameters.getMaximumStepReach();
      if (solePositionInParentZUpFrame.getZ() < -Math.abs(parameters.getMaximumStepZWhenForwardAndDown()))
      {
         if ((solePositionInParentZUpFrame.getX() > parameters.getMaximumStepXWhenForwardAndDown()))
         {
            rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN;
            return false;
         }

         if (sidedWidth > parameters.getMaximumStepYWhenForwardAndDown())
         {
            rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE_AND_DOWN;
            return false;
         }
         maxReach = EuclidCoreTools.norm(parameters.getMaximumStepXWhenForwardAndDown(), parameters.getMaximumStepYWhenForwardAndDown() - parameters.getIdealFootstepWidth());
      }

      double widthRelativeToIdeal = solePositionInParentZUpFrame.getY() - robotSide.negateIfRightSide(parameters.getIdealFootstepWidth());
      double stepReach = EuclidCoreTools.norm(solePositionInParentZUpFrame.getX(), widthRelativeToIdeal);
      if (stepReach > parameters.getMaximumStepReach())
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR;
         return false;
      }

      if (solePositionInParentZUpFrame.getZ() > parameters.getMaximumStepZWhenSteppingUp())
      {
         if (stepReach > parameters.getMaximumStepReachWhenSteppingUp())
         {
            rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_AND_HIGH;
            return false;
         }

         if (sidedWidth > parameters.getMaximumStepReachWhenSteppingUp())
         {
            rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE_AND_HIGH;
            return false;
         }
         maxReach = parameters.getMaximumStepReachWhenSteppingUp();
      }

      double stepReach3D = EuclidCoreTools.norm(stepReach, solePositionInParentZUpFrame.getZ());
      double maxInterpolationFactor = Math.max(stepReach3D / maxReach, Math.abs(solePositionInParentZUpFrame.getZ() / parameters.getMaximumStepZ()));
      maxInterpolationFactor = Math.min(maxInterpolationFactor, 1.0);
      double maxYaw = InterpolationTools.linearInterpolate(parameters.getMaximumStepYaw(), (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMaximumStepYaw(),
                                                           maxInterpolationFactor);
      double minYaw = InterpolationTools.linearInterpolate(parameters.getMinimumStepYaw(), (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMinimumStepYaw(),
                                                           maxInterpolationFactor);
      double yawDelta = AngleTools.computeAngleDifferenceMinusPiToPi(nodeToCheck.getYaw(), previousNode.getYaw());
      if (!MathTools.intervalContains(robotSide.negateIfRightSide(yawDelta), minYaw, maxYaw))
      {
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH;
         return false;
      }

      if (graph != null)
      {
         FootstepNode grandparentNode = graph.getParentNode(previousNode);
         if (grandparentNode != null)
         {
            FootstepNodeSnapData grandparentSnapData = snapper.snapFootstepNode(grandparentNode);
            if (grandparentSnapData == null)
               return true;

            Point3D grandparentPosition = new Point3D(grandparentNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
            grandparentSnapData.getSnapTransform().transform(grandparentPosition);
            double grandparentTranslationScaleFactor = parameters.getTranslationScaleFromGrandparentNode();

            Point3D nodePosition = new Point3D(nodeToCheck.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
            snapData.getSnapTransform().transform(nodePosition);

            double heightChangeFromGrandparentNode = nodePosition.getZ() - grandparentPosition.getZ();
            double translationFromGrandparentNode = EuclidCoreTools.norm(nodePosition.getX() - grandparentPosition.getX(),
                                                                         nodePosition.getY() - grandparentPosition.getY());

            boolean largeStepUp = heightChangeFromGrandparentNode > parameters.getMaximumStepZWhenSteppingUp();
            boolean largeStepDown = heightChangeFromGrandparentNode < -parameters.getMaximumStepZWhenForwardAndDown();

            if (largeStepUp && translationFromGrandparentNode > grandparentTranslationScaleFactor * parameters.getMaximumStepReachWhenSteppingUp())
            {
               rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_AND_HIGH;
               return false;
            }

            if (largeStepDown && translationFromGrandparentNode > grandparentTranslationScaleFactor * parameters.getMaximumStepXWhenForwardAndDown())
            {
               rejectionReason = BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN;
               return false;
            }
         }
      }

      return true;
   }

   @Override
   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason;
   }
}
