package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SnapBasedNodeChecker implements FootstepNodeChecker
{
   private static final double defaultMaxStepHeightChange = 0.2;
   private static final double defaultMinPercentageOfFoothold = 0.95;

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepNodeSnapper snapper;

   private final YoDouble maxStepHeightChange;
   private final YoDouble minPercentageFoothold;

   public SnapBasedNodeChecker(SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapper snapper, YoVariableRegistry registry)
   {
      this.footPolygons = footPolygons;
      this.snapper = snapper;

      maxStepHeightChange = new YoDouble("maxStepHeightChange", registry);
      minPercentageFoothold = new YoDouble("minPercentageFoothold", registry);

      maxStepHeightChange.set(defaultMaxStepHeightChange);
      minPercentageFoothold.set(defaultMinPercentageOfFoothold);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      snapper.setPlanarRegions(planarRegions);
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      if (snapTransform.containsNaN())
         return false;

      ConvexPolygon2D footholdAfterSnap = snapData.getCroppedFoothold();
      double area = footholdAfterSnap.getArea();
      double footArea = footPolygons.get(node.getRobotSide()).getArea();
      if (area < minPercentageFoothold.getDoubleValue() * footArea)
         return false;

      if(previousNode == null)
         return true;

      FootstepNodeSnapData previousNodeSnapData = snapper.snapFootstepNode(previousNode);
      RigidBodyTransform previousSnapTransform = previousNodeSnapData.getSnapTransform();
      double heightChange = Math.abs(snapTransform.getTranslationZ() - previousSnapTransform.getTranslationZ());
      if (heightChange > maxStepHeightChange.getDoubleValue())
         return false;

      return true;
   }
}
