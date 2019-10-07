package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class FootstepNodeSnapAndWiggler extends FootstepNodeSnapper
{
   private final List<BipedalFootstepPlannerListener> listeners = new ArrayList<>();
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeBodyCollisionDetector collisionDetector;

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final PlanarRegion planarRegionToPack = new PlanarRegion();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   public FootstepNodeSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters)
   {
      this(footPolygonsInSoleFrame, parameters, null);
   }

   public FootstepNodeSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters, FootstepNodeBodyCollisionDetector collisionDetector)
   {
      this.parameters = parameters;
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.collisionDetector = collisionDetector;
   }

   public void addPlannerListener(BipedalFootstepPlannerListener listener)
   {
      listeners.add(listener);
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

      if (shiftFootToAvoidBodyCollision(footstepNode, snapTransform))
      {
         return doShiftFromNearCollision(footstepNode, snapTransform);
      }
      else
      {
         return doSnapAndWiggle(footstepNode, snapTransform);
      }
   }

   private FootstepNodeSnapData doShiftFromNearCollision(FootstepNode footstepNode, RigidBodyTransform snapTransform)
   {
      BodyCollisionData collisionData = collisionDetector.checkForCollision(footstepNode, snapTransform.getTranslationZ());
      double distanceOfClosestPointInFront = collisionData.getDistanceOfClosestPointInFront();
      double distanceOfClosestPointInBack = collisionData.getDistanceOfClosestPointInBack();

      double translationX;
      double maximumTranslationX = 0.5 * LatticeNode.gridSizeXY;
      if(Double.isNaN(distanceOfClosestPointInFront))
      {
         translationX = maximumTranslationX;
      }
      else if(Double.isNaN(distanceOfClosestPointInBack))
      {
         translationX = - maximumTranslationX;
      }
      else
      {
         translationX = 0.5 * (distanceOfClosestPointInFront - distanceOfClosestPointInBack);
      }

      snapTransform.appendTranslation(translationX, 0.0, 0.0);
      return new FootstepNodeSnapData(snapTransform);
   }

   private boolean shiftFootToAvoidBodyCollision(FootstepNode footstepNode, RigidBodyTransform snapTransform)
   {
      if(collisionDetector == null || !parameters.checkForBodyBoxCollisions())
         return false;

      BodyCollisionData collisionData = collisionDetector.checkForCollision(footstepNode, snapTransform.getTranslationZ());
      return !Double.isNaN(collisionData.getDistanceOfClosestPointInFront()) || !Double.isNaN(collisionData.getDistanceOfClosestPointInBack());
   }

   private FootstepNodeSnapData doSnapAndWiggle(FootstepNode footstepNode, RigidBodyTransform snapTransform)
   {
      ConvexPolygon2D footholdPolygonInLocalFrame = FootstepNodeSnappingTools
            .getConvexHullOfPolygonIntersections(planarRegionToPack, footPolygon, snapTransform);
      if (footholdPolygonInLocalFrame.isEmpty())
         return FootstepNodeSnapData.emptyData();

      RigidBodyTransform wiggleTransformLocalToLocal = getWiggleTransformInPlanarRegionFrame(footholdPolygonInLocalFrame);

      if (wiggleTransformLocalToLocal == null)
      {
         if (parameters.getRejectIfCannotFullyWiggleInside())
         {
            rejectNode(footstepNode, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_WIGGLE_INSIDE);
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
                     rejectNode(node, BipedalFootstepPlannerNodeRejectionReason.TOO_MUCH_PENETRATION_AFTER_WIGGLE);
                     return true;
                  }
               }
            }
         }
      }

      return false;
   }

   private void rejectNode(FootstepNode nodeToExpand, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      for (BipedalFootstepPlannerListener listener : listeners)
         listener.rejectNode(nodeToExpand, null, reason);
   }
}
