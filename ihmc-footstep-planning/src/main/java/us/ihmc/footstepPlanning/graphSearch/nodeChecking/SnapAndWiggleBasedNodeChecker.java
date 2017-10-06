package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.*;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.PlanarRegionBaseOfCliffAvoider;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SnapAndWiggleBasedNodeChecker implements FootstepNodeChecker
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private BipedalFootstepPlannerListener listener;
   private PlanarRegionsList planarRegionsList;
   private FootstepNodeSnapAndWiggler snapAndWiggler;
   private BipedalFootstepPlannerParameters parameters;
   private final PlanarRegionBaseOfCliffAvoider baseOfCliffAvoider;
   private SideDependentList<ConvexPolygon2D> controllerPolygonsInSoleFrame;
   private final ConvexPolygon2D footholdIntersection = new ConvexPolygon2D();

   private final YoDouble footArea = new YoDouble("footArea", registry);
   private final YoDouble totalArea = new YoDouble("totalArea", registry);
   private final YoDouble stepReach = new YoDouble("stepReach", registry);

   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TransformReferenceFrame parentSoleFrame = new TransformReferenceFrame("parentSole", ReferenceFrame.getWorldFrame());
   private final ZUpFrame parentSoleZupFrame = new ZUpFrame(worldFrame, parentSoleFrame, "parentSoleZupFrame");
   private final TransformReferenceFrame nodeSoleFrame = new TransformReferenceFrame("nodeSole", ReferenceFrame.getWorldFrame());
   private final FramePoint3D solePositionInParentZUpFrame = new FramePoint3D(parentSoleZupFrame);

   public SnapAndWiggleBasedNodeChecker(BipedalFootstepPlannerParameters parameters,
                                        YoGraphicsListRegistry graphicsListRegistry)
   {
      this.snapAndWiggler = new FootstepNodeSnapAndWiggler(parameters);
      this.parameters = parameters;
      this.baseOfCliffAvoider = new PlanarRegionBaseOfCliffAvoider(registry, graphicsListRegistry);
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      this.snapAndWiggler.setFootPolygonsInSoleFrame(footPolygonsInSoleFrame);
      this.controllerPolygonsInSoleFrame = footPolygonsInSoleFrame;
   }

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.snapAndWiggler.setListener(listener);
      this.listener = listener;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      this.snapAndWiggler.setPlanarRegions(planarRegionsList);

      if (listener != null)
      {
         listener.planarRegionsListSet(planarRegionsList);
      }
   }

   @Override
   public boolean isNodeValid(FootstepNode nodeToExpand, FootstepNode previousNode)
   {
      RigidBodyTransform snapTransform = snapAndWiggler.snapFootstepNode(nodeToExpand, footholdIntersection);

      if (snapTransform == null)
         return false;

      RigidBodyTransform snappedSoleTransform = BipedalFootstepPlannerNodeUtils.getSnappedSoleTransform(nodeToExpand, snapTransform);
      baseOfCliffAvoider.shiftAwayFromCliffBottoms(parameters, planarRegionsList, snappedSoleTransform);

      boolean isEnoughArea = checkIfEnoughArea(nodeToExpand, footholdIntersection);
      if (!isEnoughArea)
         return false;

      if(previousNode == null)
         return true;

      RigidBodyTransform previousSnapTransform = snapAndWiggler.snapFootstepNode(previousNode, null);
      RigidBodyTransform previousSnappedSoleTransform = BipedalFootstepPlannerNodeUtils.getSnappedSoleTransform(previousNode, previousSnapTransform);
      boolean goodFootstep = checkIfGoodFootstep(nodeToExpand, snappedSoleTransform, previousSnappedSoleTransform);
      if (!goodFootstep)
         return false;

      notifyListenerNodeUnderConsiderationWasSuccessful(nodeToExpand);

      return true;
   }

   private boolean checkIfEnoughArea(FootstepNode nodeToExpand, ConvexPolygon2D footholdIntersection)
   {
      totalArea.set(footholdIntersection.getArea());

      if (totalArea.getDoubleValue() < parameters.getMinimumFootholdPercent() * footArea.getDoubleValue())
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
         return false;
      }

      return true;
   }

   private boolean checkIfGoodFootstep(FootstepNode nodeToExpand, RigidBodyTransform soleTransform, RigidBodyTransform previousSoleTransform)
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
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH);
         return false;
      }

      double maximumStepWidth = parameters.getMaximumStepWidth();

      if (robotSide.negateIfRightSide(solePositionInParentZUpFrame.getY()) > maximumStepWidth)
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE);
         return false;
      }

      double minimumStepLength = parameters.getMinimumStepLength();
      if (solePositionInParentZUpFrame.getX() < minimumStepLength)
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH);
         return false;
      }

      if (Math.abs(solePositionInParentZUpFrame.getZ()) > parameters.getMaximumStepZ())
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW);
         return false;
      }

      if ((solePositionInParentZUpFrame.getX() > parameters.getMaximumStepXWhenForwardAndDown())
            && (solePositionInParentZUpFrame.getZ() < -Math.abs(parameters.getMaximumStepZWhenForwardAndDown())))
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN);
         return false;
      }

      stepReach.set(getXYLength(solePositionInParentZUpFrame));
      if (stepReach.getDoubleValue() > parameters.getMaximumStepReach())
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR);
         return false;
      }

      return true;
   }

   private double getXYLength(FramePoint3D point)
   {
      return Math.sqrt(point.getX() * point.getX() + point.getY() * point.getY());
   }

   private void notifyListenerNodeUnderConsiderationWasSuccessful(FootstepNode node)
   {
      if (listener != null)
      {
         listener.nodeUnderConsiderationWasSuccessful(node);
      }
   }

   private void notifyListenerNodeUnderConsideration(FootstepNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeUnderConsideration(nodeToExpand);
      }
   }

   private void notifyListenerNodeUnderConsiderationWasRejected(FootstepNode nodeToExpand, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (listener != null)
      {
         listener.nodeUnderConsiderationWasRejected(nodeToExpand, reason);
      }
   }
}
