package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeChecker;
import us.ihmc.footstepPlanning.aStar.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class SnapBasedNodeChecker implements FootstepNodeChecker
{
   private static final double maxStepHeightChange = 0.2;
   private static final double minPercentageOfFoothold = 0.95;

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepNodeSnapper snapper;
   private final ConvexPolygon2D footholdAfterSnap;

   public SnapBasedNodeChecker(SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapper snapper)
   {
      this.footPolygons = footPolygons;
      this.snapper = snapper;
      this.footholdAfterSnap = new ConvexPolygon2D();
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      snapper.setPlanarRegions(planarRegions);
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      RigidBodyTransform snapTransform = snapper.snapFootstepNode(node, footholdAfterSnap);
      if (snapTransform == null)
         return false;

      double area = footholdAfterSnap.getArea();
      double footArea = footPolygons.get(node.getRobotSide()).getArea();
      if (area < minPercentageOfFoothold * footArea)
         return false;

      if(previousNode == null)
         return true;

      RigidBodyTransform previousSnapTransform = snapper.snapFootstepNode(previousNode, null);
      double heightChange = Math.abs(snapTransform.getTranslationZ() - previousSnapTransform.getTranslationZ());
      if (heightChange > maxStepHeightChange)
         return false;

      return true;
   }
}
