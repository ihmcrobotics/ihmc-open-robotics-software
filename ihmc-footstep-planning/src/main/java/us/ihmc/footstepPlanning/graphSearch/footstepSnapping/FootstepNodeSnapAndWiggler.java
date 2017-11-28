package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepNodeSnapAndWiggler extends FootstepNodeSnapper
{
   private final BipedalFootstepPlannerListener listener;
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final FootstepPlannerParameters parameters;

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final PlanarRegion planarRegionToPack = new PlanarRegion();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   public FootstepNodeSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParameters parameters,
                                     BipedalFootstepPlannerListener listener)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.parameters = parameters;
      this.listener = listener;
   }

   @Override
   public FootstepNodeSnapData snapInternal(FootstepNode footstepNode)
   {
      FootstepNodeTools.getFootPolygon(footstepNode, footPolygonsInSoleFrame.get(footstepNode.getRobotSide()), footPolygon);
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, planarRegionToPack);

      if (snapTransform == null)
         return FootstepNodeSnapData.emptyData();

      ConvexPolygon2D footholdPolygonInLocalFrame = FootstepNodeSnappingTools.getConvexHullOfPolygonIntersections(planarRegionToPack, footPolygon, snapTransform);
      if (footholdPolygonInLocalFrame.isEmpty())
         return FootstepNodeSnapData.emptyData();

      RigidBodyTransform wiggleTransformLocalToLocal = getWiggleTransformInPlanarRegionFrame(footholdPolygonInLocalFrame);
      
      if (wiggleTransformLocalToLocal == null)
      {
         if (parameters.getRejectIfCannotFullyWiggleInside())
         {
            notifyListenerNodeUnderConsiderationWasRejected(footstepNode, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_WIGGLE_INSIDE);
            return FootstepNodeSnapData.emptyData();
         }
         else
         {
            FootstepNodeSnappingTools.changeFromPlanarRegionToSoleFrame(planarRegionToPack, footstepNode, snapTransform, footholdPolygonInLocalFrame);
            return new FootstepNodeSnapData(snapTransform, footholdPolygonInLocalFrame);
         }
      }

      RigidBodyTransform wiggleTransformWorldToWorld = getWiggleTransformInWorldFrame(wiggleTransformLocalToLocal);
      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform(wiggleTransformWorldToWorld);
      snapAndWiggleTransform.multiply(snapTransform);

      // Ensure polygon will be completely above the planarRegions with this snap and wiggle:
      ConvexPolygon2D footPolygonInWorld = new ConvexPolygon2D(footholdPolygonInLocalFrame);
      footPolygonInWorld.applyTransformAndProjectToXYPlane(snapAndWiggleTransform);

      List<PlanarRegion> planarRegionsIntersectingSnappedAndWiggledPolygon = planarRegionsList.findPlanarRegionsIntersectingPolygon(footPolygonInWorld);

      if (checkForTooMuchPenetrationAfterWiggle(footstepNode, planarRegionToPack, footPolygonInWorld,
                                                planarRegionsIntersectingSnappedAndWiggledPolygon))
         return FootstepNodeSnapData.emptyData();

      ConvexPolygon2D wiggledFootholdPolygonInLocalFrame = FootstepNodeSnappingTools.getConvexHullOfPolygonIntersections(planarRegionToPack, footPolygon, snapAndWiggleTransform);
      FootstepNodeSnappingTools.changeFromPlanarRegionToSoleFrame(planarRegionToPack, footstepNode, snapAndWiggleTransform, wiggledFootholdPolygonInLocalFrame);

      return new FootstepNodeSnapData(snapAndWiggleTransform, wiggledFootholdPolygonInLocalFrame);
   }

   private RigidBodyTransform getWiggleTransformInPlanarRegionFrame(ConvexPolygon2D footholdPolygon)
   {
      updateWiggleParameters();

      if (parameters.getWiggleIntoConvexHullOfPlanarRegions())
         return PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footholdPolygon, planarRegionToPack, wiggleParameters);
      else
         return PolygonWiggler.wigglePolygonIntoRegion(footholdPolygon, planarRegionToPack, wiggleParameters);
   }

   private void updateWiggleParameters()
   {
      wiggleParameters.deltaInside = parameters.getWiggleInsideDelta();
      wiggleParameters.maxX = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaximumYawWiggle();
   }

   private RigidBodyTransform getWiggleTransformInWorldFrame(RigidBodyTransform wiggleTransformLocalToLocal)
   {
      RigidBodyTransform wiggleTransformWorldToWorld = new RigidBodyTransform();
      RigidBodyTransform localToWorld = new RigidBodyTransform();
      planarRegionToPack.getTransformToWorld(localToWorld);
      RigidBodyTransform worldToLocal = new RigidBodyTransform(localToWorld);
      worldToLocal.invert();

      wiggleTransformWorldToWorld.set(localToWorld);
      wiggleTransformWorldToWorld.multiply(wiggleTransformLocalToLocal);
      wiggleTransformWorldToWorld.multiply(worldToLocal);
      return wiggleTransformWorldToWorld;
   }

   private boolean checkForTooMuchPenetrationAfterWiggle(FootstepNode node, PlanarRegion highestElevationPlanarRegion,
                                                         ConvexPolygon2D footPolygonInWorld,
                                                         List<PlanarRegion> planarRegionsIntersectingSnappedAndWiggledPolygon)
   {
      ArrayList<ConvexPolygon2D> intersectionsInPlaneFrameToPack = new ArrayList<>();
      RigidBodyTransform transformToWorldFromIntersectingPlanarRegion = new RigidBodyTransform();

      if (planarRegionsIntersectingSnappedAndWiggledPolygon != null)
      {
         for (PlanarRegion planarRegionIntersectingSnappedAndWiggledPolygon : planarRegionsIntersectingSnappedAndWiggledPolygon)
         {
            planarRegionIntersectingSnappedAndWiggledPolygon.getTransformToWorld(transformToWorldFromIntersectingPlanarRegion);
            intersectionsInPlaneFrameToPack.clear();
            planarRegionIntersectingSnappedAndWiggledPolygon.getPolygonIntersectionsWhenProjectedVertically(footPolygonInWorld,
                                                                                                            intersectionsInPlaneFrameToPack);

            // If any points are above the plane of the planarRegionToPack, then this is stepping into a v type problem.
            for (ConvexPolygon2D intersectionPolygon : intersectionsInPlaneFrameToPack)
            {
               int numberOfVertices = intersectionPolygon.getNumberOfVertices();
               for (int i = 0; i < numberOfVertices; i++)
               {
                  Point2DReadOnly vertex2d = intersectionPolygon.getVertex(i);
                  Point3D vertex3dInWorld = new Point3D(vertex2d.getX(), vertex2d.getY(), 0.0);
                  transformToWorldFromIntersectingPlanarRegion.transform(vertex3dInWorld);

                  double planeZGivenXY = highestElevationPlanarRegion.getPlaneZGivenXY(vertex3dInWorld.getX(), vertex3dInWorld.getY());

                  double zPenetration = vertex3dInWorld.getZ() - planeZGivenXY;

                  if (zPenetration > parameters.getMaximumZPenetrationOnValleyRegions())
                  {
                     notifyListenerNodeUnderConsiderationWasRejected(node, BipedalFootstepPlannerNodeRejectionReason.TOO_MUCH_PENETRATION_AFTER_WIGGLE);
                     return true;
                  }
               }
            }
         }
      }
      return false;
   }

   private void notifyListenerNodeUnderConsiderationWasRejected(FootstepNode nodeToExpand, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (listener != null)
      {
         listener.nodeUnderConsiderationWasRejected(nodeToExpand, reason);
      }
   }
}
