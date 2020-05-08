package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.ConcavePolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class FootstepNodeSnapAndWiggler extends FootstepNodeSnapper
{
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final FootstepPlannerParametersReadOnly parameters;

   private final ConcavePolygonWiggler concavePolygonWiggler = new ConcavePolygonWiggler();
   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final PlanarRegion planarRegionToPack = new PlanarRegion();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   public FootstepNodeSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.parameters = parameters;
   }

   @Override
   public FootstepNodeSnapData snapInternal(FootstepNode footstepNode)
   {
      if (!hasPlanarRegions())
      {
         return FootstepNodeSnapData.identityData();
      }

      FootstepNodeTools.getFootPolygon(footstepNode, footPolygonsInSoleFrame.get(footstepNode.getRobotSide()), footPolygon);
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, planarRegionToPack);

      if (snapTransform == null)
      {
         return FootstepNodeSnapData.emptyData();
      }

      return doSnapAndWiggle(footstepNode, snapTransform);
   }

   private FootstepNodeSnapData doSnapAndWiggle(FootstepNode footstepNode, RigidBodyTransform snapTransform)
   {
      ConvexPolygon2D footholdPolygonInLocalFrame = FootstepNodeSnappingTools
            .getConvexHullOfPolygonIntersections(planarRegionToPack, footPolygon, snapTransform);
      if (footholdPolygonInLocalFrame.isEmpty())
         return FootstepNodeSnapData.emptyData();

      boolean doWiggle = false;
      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = footPolygon.getVertex(i);
         if (planarRegionToPack.getConvexHull().signedDistance(vertex) > - parameters.getWiggleInsideDelta())
         {
            doWiggle = true;
            break;
         }
      }

      RigidBodyTransform wiggleTransformLocalToLocal = null;
      if (doWiggle)
      {
         wiggleTransformLocalToLocal = getWiggleTransformInPlanarRegionFrame(footholdPolygonInLocalFrame);
      }

      if (wiggleTransformLocalToLocal == null)
      {
         FootstepNodeSnappingTools.changeFromPlanarRegionToSoleFrame(planarRegionToPack, footstepNode, snapTransform, footholdPolygonInLocalFrame);
         return new FootstepNodeSnapData(snapTransform, footholdPolygonInLocalFrame);
      }

      RigidBodyTransform wiggleTransformWorldToWorld = getWiggleTransformInWorldFrame(wiggleTransformLocalToLocal);
      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform(wiggleTransformWorldToWorld);
      snapAndWiggleTransform.multiply(snapTransform);

      // Ensure polygon will be completely above the planarRegions with this snap and wiggle:
      ConvexPolygon2D footPolygonInWorld = new ConvexPolygon2D(footholdPolygonInLocalFrame);
      footPolygonInWorld.applyTransform(snapAndWiggleTransform, false);

      List<PlanarRegion> planarRegionsIntersectingSnappedAndWiggledPolygon = PlanarRegionTools
            .findPlanarRegionsIntersectingPolygon(footPolygonInWorld, planarRegionsList);

      if (checkForTooMuchPenetrationAfterWiggle(footstepNode, planarRegionToPack, footPolygonInWorld, planarRegionsIntersectingSnappedAndWiggledPolygon))
         return FootstepNodeSnapData.emptyData();

      ConvexPolygon2D wiggledFootholdPolygonInLocalFrame = FootstepNodeSnappingTools
            .getConvexHullOfPolygonIntersections(planarRegionToPack, footPolygon, snapAndWiggleTransform);
      FootstepNodeSnappingTools.changeFromPlanarRegionToSoleFrame(planarRegionToPack, footstepNode, snapAndWiggleTransform, wiggledFootholdPolygonInLocalFrame);
      checkForExtraContactPoints(wiggledFootholdPolygonInLocalFrame, footPolygonsInSoleFrame.get(footstepNode.getRobotSide()));

      return new FootstepNodeSnapData(snapAndWiggleTransform, wiggledFootholdPolygonInLocalFrame);
   }

   private static void checkForExtraContactPoints(ConvexPolygon2D croppedFootPolygon, ConvexPolygon2D defaultFootPolygon)
   {
      if(croppedFootPolygon.getNumberOfVertices() <= 4)
         return;

      double epsilonContactPointCheck = 5e-3;
      outerLoop: for (int i = 0; i < defaultFootPolygon.getNumberOfVertices(); i++)
      {
         for (int j = 0; j < croppedFootPolygon.getNumberOfVertices(); j++)
         {
            if(croppedFootPolygon.getVertex(j).epsilonEquals(defaultFootPolygon.getVertex(i), epsilonContactPointCheck))
               continue outerLoop;
         }

         return;
      }

      // contains all default contact points, remove any extra
      outerLoop: for (int i = 0; i < croppedFootPolygon.getNumberOfVertices(); i++)
      {
         for (int j = 0; j < defaultFootPolygon.getNumberOfVertices(); j++)
         {
            if(croppedFootPolygon.getVertex(i).epsilonEquals(defaultFootPolygon.getVertex(j), epsilonContactPointCheck))
               continue outerLoop;
         }

         croppedFootPolygon.removeVertex(i);
         croppedFootPolygon.update();
         i--;
      }
   }

   RigidBodyTransform getWiggleTransformInPlanarRegionFrame(ConvexPolygon2D footholdPolygon)
   {
      updateWiggleParameters();

      if (parameters.getEnableConcaveHullWiggler() && !planarRegionToPack.getConcaveHull().isEmpty())
         return concavePolygonWiggler.wigglePolygon(footholdPolygon, Vertex2DSupplier.asVertex2DSupplier(planarRegionToPack.getConcaveHull()), wiggleParameters);
      else
         return PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footholdPolygon, planarRegionToPack, wiggleParameters);
   }

   private void updateWiggleParameters()
   {
      wiggleParameters.deltaInside = parameters.getWiggleInsideDelta();
      wiggleParameters.maxX = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minX = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minY = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaximumYawWiggle();
      wiggleParameters.minYaw = -parameters.getMaximumYawWiggle();
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

   private boolean checkForTooMuchPenetrationAfterWiggle(FootstepNode node, PlanarRegion highestElevationPlanarRegion, ConvexPolygon2D footPolygonInWorld,
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
            planarRegionIntersectingSnappedAndWiggledPolygon
                  .getPolygonIntersectionsWhenProjectedVertically(footPolygonInWorld, intersectionsInPlaneFrameToPack);

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
                     return true;
                  }
               }
            }
         }
      }

      return false;
   }
}
