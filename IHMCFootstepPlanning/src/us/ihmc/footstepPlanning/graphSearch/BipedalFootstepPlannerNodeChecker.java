package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BipedalFootstepPlannerNodeChecker
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private BipedalFootstepPlannerNode startNode;
   private BipedalFootstepPlannerListener listener;
   private PlanarRegionsList planarRegionsList;
   private BipedalFootstepPlannerSnapAndWiggler snapAndWiggler;
   private BipedalFootstepPlannerParameters parameters;
   private final PlanarRegionBaseOfCliffAvoider baseOfCliffAvoider;
   private SideDependentList<ConvexPolygon2D> controllerPolygonsInSoleFrame;

   private final YoDouble footArea = new YoDouble("footArea", registry);
   private final YoDouble totalArea = new YoDouble("totalArea", registry);
   private final YoDouble stepReach = new YoDouble("stepReach", registry);

   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TransformReferenceFrame parentSoleFrame = new TransformReferenceFrame("parentSole", ReferenceFrame.getWorldFrame());
   private final ZUpFrame parentSoleZupFrame = new ZUpFrame(worldFrame, parentSoleFrame, "parentSoleZupFrame");
   private final TransformReferenceFrame nodeSoleFrame = new TransformReferenceFrame("nodeSole", ReferenceFrame.getWorldFrame());
   private final FramePoint3D solePositionInParentZUpFrame = new FramePoint3D(parentSoleZupFrame);

   public BipedalFootstepPlannerNodeChecker(BipedalFootstepPlannerParameters parameters,
                                            YoGraphicsListRegistry graphicsListRegistry)
   {
      this.snapAndWiggler = new BipedalFootstepPlannerSnapAndWiggler(parameters);
      this.parameters = parameters;
      this.baseOfCliffAvoider = new PlanarRegionBaseOfCliffAvoider(registry, graphicsListRegistry);
   }

   public void setStartNode(BipedalFootstepPlannerNode startNode)
   {
      this.startNode = startNode;
   }

   public void setFeetPolygons(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, SideDependentList<ConvexPolygon2D> controllerPolygonsInSoleFrame)
   {
      this.snapAndWiggler.setFootPolygonsInSoleFrame(footPolygonsInSoleFrame);
      this.controllerPolygonsInSoleFrame = controllerPolygonsInSoleFrame;
   }

   public void setBipedalFootstepPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.snapAndWiggler.setListener(listener);
      this.listener = listener;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      this.snapAndWiggler.setPlanarRegionsList(planarRegionsList);

      if (listener != null)
      {
         listener.planarRegionsListSet(planarRegionsList);
      }
   }

   public boolean snapNodeAndCheckIfAcceptableToExpand(BipedalFootstepPlannerNode nodeToExpand)
   {
      // Make sure popped node is a good one and can be expanded...
      boolean snapSucceded = snapToPlanarRegionAndCheckIfGoodSnap(nodeToExpand);
      if (!snapSucceded)
         return false;

      boolean goodFootstep = checkIfGoodFootstep(nodeToExpand);
      if (!goodFootstep)
         return false;

      boolean differentFromParent = checkIfDifferentFromGrandParent(nodeToExpand);
      {
         if (!differentFromParent)
            return false;
      }

      notifyListenerNodeUnderConsiderationWasSuccessful(nodeToExpand);

      return true;
   }

   private boolean snapToPlanarRegionAndCheckIfGoodSnap(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (planarRegionsList != null)
      {
         PlanarRegion planarRegion = new PlanarRegion();
         RigidBodyTransform nodeToExpandSnapTransform = snapAndWiggler.getSnapAndWiggleTransform(parameters.getWiggleInsideDelta(), nodeToExpand, planarRegion);

         if (nodeToExpandSnapTransform == null)
         {
            return false;
         }

         RigidBodyTransform snappedSoleTransform = BipedalFootstepPlannerNodeUtils.getSnappedSoleTransform(nodeToExpand, nodeToExpandSnapTransform);
         baseOfCliffAvoider.shiftAwayFromCliffBottoms(parameters, planarRegionsList, snappedSoleTransform);

         boolean isEnoughArea = checkIfEnoughArea(nodeToExpand, planarRegion);
         if (!isEnoughArea)
            return false;
      }

      return true;
   }

   private boolean checkIfEnoughArea(BipedalFootstepPlannerNode nodeToExpand, PlanarRegion planarRegion)
   {
      RigidBodyTransform nodeToExpandTransform = new RigidBodyTransform();
      nodeToExpand.getSoleTransform(nodeToExpandTransform);

      ConvexPolygon2D snappedPolygon = controllerPolygonsInSoleFrame.get(nodeToExpand.getRobotSide());
      snappedPolygon.update();
      footArea.set(snappedPolygon.getArea());

      ConvexPolygon2D footholdPolygon = new ConvexPolygon2D();
      totalArea.set(planarRegion.getPolygonIntersectionAreaWhenSnapped(snappedPolygon, nodeToExpandTransform, footholdPolygon));

      nodeToExpand.setPercentageOfFoothold(totalArea.getDoubleValue() / footArea.getDoubleValue());

      if (nodeToExpand.isPartialFoothold())
      {
         nodeToExpand.setPartialFootholdPolygon(footholdPolygon);
      }

      if (totalArea.getDoubleValue() < parameters.getMinimumFootholdPercent() * footArea.getDoubleValue())
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
         return false;
      }

      return true;
   }

   private boolean checkIfGoodFootstep(BipedalFootstepPlannerNode nodeToExpand)
   {
      BipedalFootstepPlannerNode parentNode = nodeToExpand.getParentNode();
      if (parentNode == null)
         return true;

      parentNode.getSoleTransform(transform);

      parentSoleFrame.setTransformAndUpdate(transform);
      parentSoleZupFrame.update();

      nodeToExpand.getSoleTransform(transform);
      nodeSoleFrame.setTransformAndUpdate(transform);

      solePositionInParentZUpFrame.setToZero(nodeSoleFrame);
      solePositionInParentZUpFrame.changeFrame(parentSoleZupFrame);

      double minimumStepWidth = parameters.getMinimumStepWidth();

      RobotSide robotSide = nodeToExpand.getRobotSide();
      if (((robotSide == RobotSide.LEFT) && (solePositionInParentZUpFrame.getY() < minimumStepWidth))
            || ((robotSide == RobotSide.RIGHT) && (solePositionInParentZUpFrame.getY() > -minimumStepWidth)))
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH);
         return false;
      }

      double maximumStepWidth = parameters.getMaximumStepWidth();

      if (((robotSide == RobotSide.LEFT) && (solePositionInParentZUpFrame.getY() > maximumStepWidth))
            || ((robotSide == RobotSide.RIGHT) && (solePositionInParentZUpFrame.getY() < -maximumStepWidth)))
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

   private boolean checkIfDifferentFromGrandParent(BipedalFootstepPlannerNode nodeToExpand)
   {
      // OK to step in place if at goal.
      if (nodeToExpand.isAtGoal())
         return true;

      BipedalFootstepPlannerNode parentNode = nodeToExpand.getParentNode();
      if (parentNode == null)
         return true;

      BipedalFootstepPlannerNode grandParentNode = parentNode.getParentNode();
      if (grandParentNode == null)
      {
         //TODO: How to check in place step when first step?
         return true;
      }

      if (grandParentNode.equals(nodeToExpand))
      {
         notifyListenerNodeUnderConsiderationWasRejected(nodeToExpand, BipedalFootstepPlannerNodeRejectionReason.STEP_IN_PLACE);
         return false;
      }

      return true;
   }

   private void notifyListenerNodeUnderConsiderationWasSuccessful(BipedalFootstepPlannerNode node)
   {
      if (listener != null)
      {
         listener.nodeUnderConsiderationWasSuccessful(node);
      }
   }

   private void notifyListenerNodeUnderConsideration(BipedalFootstepPlannerNode nodeToExpand)
   {
      if (listener != null)
      {
         listener.nodeUnderConsideration(nodeToExpand);
      }
   }

   private void notifyListenerNodeUnderConsiderationWasRejected(BipedalFootstepPlannerNode nodeToExpand, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (listener != null)
      {
         listener.nodeUnderConsiderationWasRejected(nodeToExpand, reason);
      }
   }
}
