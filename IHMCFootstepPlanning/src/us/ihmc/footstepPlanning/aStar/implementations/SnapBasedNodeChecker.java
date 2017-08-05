package us.ihmc.footstepPlanning.aStar.implementations;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeChecker;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SnapBasedNodeChecker implements FootstepNodeChecker
{
   private PlanarRegionsList planarRegions;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private static final double maxStepHeightChange = 0.2;
   private static final double minPercentageOfFoothold = 0.95;

   public SnapBasedNodeChecker(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.footPolygons = footPolygons;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegions = planarRegions;
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode)
   {
      if (planarRegions == null)
         return true;

      PlanarRegion region = new PlanarRegion();
      RigidBodyTransform snapTransform = computeSnapTransform(node, region);
      if (snapTransform == null)
         return false;

      RigidBodyTransform previousSnapTransform = computeSnapTransform(previosNode, null);
      double heightChange = Math.abs(snapTransform.getTranslationZ() - previousSnapTransform.getTranslationZ());
      if (heightChange > maxStepHeightChange)
         return false;

      ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
      ConvexPolygon2D footPolygonInWorld = getFootPolygon(node);
      region.getPolygonIntersectionsWhenProjectedVertically(footPolygonInWorld, intersections);
      if (intersections.size() != 1)
         throw new RuntimeException("Invalid Intersection");

      double area = intersections.get(0).getArea();
      double footArea = footPolygonInWorld.getArea();
      if (area < minPercentageOfFoothold * footArea)
         return false;

      return true;
   }

   private RigidBodyTransform computeSnapTransform(FootstepNode node, PlanarRegion region)
   {
      ConvexPolygon2D footPolygon = getFootPolygon(node);
      return PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegions, region);
   }

   private ConvexPolygon2D getFootPolygon(FootstepNode node)
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D(footPolygons.get(node.getRobotSide()));
      RigidBodyTransform worldToSole = new RigidBodyTransform();
      worldToSole.setRotationYawAndZeroTranslation(node.getYaw());
      worldToSole.setTranslation(node.getX(), node.getY(), 0.0);
      footPolygon.applyTransformAndProjectToXYPlane(worldToSole);
      return footPolygon;
   }

}
