package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SnapAndWiggleBasedNodeChecker extends FootstepNodeChecker
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private FootstepNodeSnapAndWiggler snapAndWiggler;
   private FootstepPlannerParameters parameters;
   private final ConvexPolygon2D footholdIntersection = new ConvexPolygon2D();

   private final YoDouble footArea = new YoDouble("footArea", registry);
   private final YoDouble totalArea = new YoDouble("totalArea", registry);
   private final YoDouble stepReach = new YoDouble("stepReach", registry);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TransformReferenceFrame parentSoleFrame = new TransformReferenceFrame("parentSole", ReferenceFrame.getWorldFrame());
   private final ZUpFrame parentSoleZupFrame = new ZUpFrame(worldFrame, parentSoleFrame, "parentSoleZupFrame");
   private final TransformReferenceFrame nodeSoleFrame = new TransformReferenceFrame("nodeSole", ReferenceFrame.getWorldFrame());
   private final FramePoint3D solePositionInParentZUpFrame = new FramePoint3D(parentSoleZupFrame);

   public SnapAndWiggleBasedNodeChecker(SideDependentList<ConvexPolygon2D> footPolygons, FootstepPlannerParameters parameters)
   {
      this.snapAndWiggler = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      this.parameters = parameters;
   }

   @Override
   public void addPlannerListener(BipedalFootstepPlannerListener listener)
   {
      snapAndWiggler.addPlannerListener(listener);
      listeners.add(listener);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      super.setPlanarRegions(planarRegionsList);
      this.snapAndWiggler.setPlanarRegions(planarRegionsList);
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode nodeToExpand, FootstepNode previousNode)
   {
      FootstepNodeSnapData snapData = snapAndWiggler.snapFootstepNode(nodeToExpand);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      if (snapTransform.containsNaN())
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return false;
      }

      RigidBodyTransform snappedSoleTransform = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransform(nodeToExpand, snapTransform, snappedSoleTransform);

      if (snappedSoleTransform.getM22() < Math.cos(parameters.getMinimumSurfaceInclineRadians()))
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);
         return false;
      }

      boolean isEnoughArea = checkIfEnoughArea(nodeToExpand, previousNode, footholdIntersection);
      if (!isEnoughArea)
         return false;

      if (previousNode == null)
         return true;

      FootstepNodeSnapData previousNodeSnapData = snapAndWiggler.snapFootstepNode(previousNode);
      RigidBodyTransform previousSnapTransform = previousNodeSnapData.getSnapTransform();
      RigidBodyTransform previousSnappedSoleTransform = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransform(previousNode, previousSnapTransform, previousSnappedSoleTransform);

      boolean goodFootstep = checkIfGoodFootstep(nodeToExpand, previousNode, snappedSoleTransform, previousSnappedSoleTransform);
      if (!goodFootstep)
         return false;

      return true;
   }

   private boolean checkIfEnoughArea(FootstepNode nodeToExpand, FootstepNode previousNode, ConvexPolygon2D footholdIntersection)
   {
      totalArea.set(footholdIntersection.getArea());

      if (totalArea.getDoubleValue() < parameters.getMinimumFootholdPercent() * footArea.getDoubleValue())
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
         return false;
      }

      return true;
   }

   private boolean checkIfGoodFootstep(FootstepNode nodeToExpand, FootstepNode previousNode, RigidBodyTransform soleTransform,
                                       RigidBodyTransform previousSoleTransform)
   {
      parentSoleFrame.setTransformAndUpdate(previousSoleTransform);
      parentSoleZupFrame.update();

      nodeSoleFrame.setTransformAndUpdate(soleTransform);

      solePositionInParentZUpFrame.setToZero(nodeSoleFrame);
      solePositionInParentZUpFrame.changeFrame(parentSoleZupFrame);

      double minimumStepWidth = parameters.getMinimumStepWidth();

      RobotSide robotSide = nodeToExpand.getRobotSide();
      if (robotSide.negateIfRightSide(solePositionInParentZUpFrame.getY()) < minimumStepWidth)
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH);
         return false;
      }

      double maximumStepWidth = parameters.getMaximumStepWidth();

      if (robotSide.negateIfRightSide(solePositionInParentZUpFrame.getY()) > maximumStepWidth)
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE);
         return false;
      }

      double minimumStepLength = parameters.getMinimumStepLength();
      if (solePositionInParentZUpFrame.getX() < minimumStepLength)
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH);
         return false;
      }

      if (Math.abs(solePositionInParentZUpFrame.getZ()) > parameters.getMaximumStepZ())
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW);
         return false;
      }

      if ((solePositionInParentZUpFrame.getX() > parameters.getMaximumStepXWhenForwardAndDown()) && (solePositionInParentZUpFrame.getZ() < -Math
            .abs(parameters.getMaximumStepZWhenForwardAndDown())))
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN);
         return false;
      }

      stepReach.set(getXYLength(solePositionInParentZUpFrame));
      if (stepReach.getDoubleValue() > parameters.getMaximumStepReach())
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR);
         return false;
      }

      if (stepReach.getDoubleValue() > parameters.getMaximumStepReachWhenSteppingUp() && solePositionInParentZUpFrame.getZ() > parameters
            .getMaximumStepZWhenSteppingUp())
      {
         rejectNode(nodeToExpand, previousNode, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_AND_HIGH);
         return false;
      }

      return true;
   }

   private double getXYLength(FramePoint3D point)
   {
      return Math.sqrt(point.getX() * point.getX() + point.getY() * point.getY());
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
      snapAndWiggler.addSnapData(startNode, new FootstepNodeSnapData(startNodeTransform));
   }
}
