package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.*;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class FootstepNodeSnapAndWiggler implements FootstepNodeSnapper
{
   private PlanarRegionsList planarRegionsList;
   private BipedalFootstepPlannerListener listener;
   private SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private BipedalFootstepPlannerParameters parameters;

   public FootstepNodeSnapAndWiggler(BipedalFootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setFootPolygonsInSoleFrame(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
   }

   public void setListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }

   public RigidBodyTransform snapFootstepNode(FootstepNode bipedalFootstepPlannerNode, ConvexPolygon2D footholdIntersectionToPack)
   {
      if (planarRegionsList == null)
      {
         throw new RuntimeException("Only call this if planarRegionsList exists. Check for null before calling.");
      }

      RobotSide nodeSide = bipedalFootstepPlannerNode.getRobotSide();
      RigidBodyTransform soleTransformBeforeSnap = new RigidBodyTransform();

      BipedalFootstepPlannerNodeUtils.getSoleTransform(bipedalFootstepPlannerNode, soleTransformBeforeSnap);
      if (!isTransformZUp(soleTransformBeforeSnap))
      {
         throw new RuntimeException("Node needs to be flat (no pitch or roll) before calling this! bipedalFootstepPlannerNode = \n"
                                          + bipedalFootstepPlannerNode);
      }

      ConvexPolygon2D currentFootPolygon = new ConvexPolygon2D(footPolygonsInSoleFrame.get(nodeSide));
      currentFootPolygon.applyTransformAndProjectToXYPlane(soleTransformBeforeSnap);

      PlanarRegion planarRegionToPack = new PlanarRegion();
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(currentFootPolygon, planarRegionsList,
                                                                                                        planarRegionToPack);
      if (snapTransform == null)
      {
         notifyListenerNodeUnderConsiderationWasRejected(bipedalFootstepPlannerNode, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return null;
      }

      if (Math.abs(snapTransform.getM22()) < parameters.getMinimumSurfaceInclineRadians())
      {
         notifyListenerNodeUnderConsiderationWasRejected(bipedalFootstepPlannerNode,
                                                         BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);
         return null;
      }

      BipedalFootstepPlannerNodeUtils.getSnappedSoleTransform(bipedalFootstepPlannerNode, snapTransform);

      WiggleParameters wiggleParameters = new WiggleParameters();
      wiggleParameters.deltaInside = parameters.getWiggleInsideDelta();
      //      parameters.minX = -0.1;
      //      parameters.maxX = 0.1;
      //      parameters.minY = -0.1;
      //      parameters.maxY = 0.1;
      //      parameters.minYaw = -0.1;
      //      parameters.maxYaw = 0.1;
      //      parameters.rotationWeight = 1.0;

      ConvexPolygon2D polygonToWiggleInRegionFrame = planarRegionToPack.snapPolygonIntoRegionAndChangeFrameToRegionFrame(currentFootPolygon, snapTransform);
      //      System.out.println("polygonToWiggleInRegionFrame = \n" + polygonToWiggleInRegionFrame);
      //      System.out.println("planarRegionToPack = \n" + planarRegionToPack);

      RigidBodyTransform wiggleTransformLocalToLocal = null;
      if (parameters.getWiggleIntoConvexHullOfPlanarRegions())
         wiggleTransformLocalToLocal = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(polygonToWiggleInRegionFrame, planarRegionToPack, wiggleParameters);
      else
         wiggleTransformLocalToLocal = PolygonWiggler.wigglePolygonIntoRegion(polygonToWiggleInRegionFrame, planarRegionToPack, wiggleParameters);

      if (wiggleTransformLocalToLocal == null)
      {
         notifyListenerNodeUnderConsiderationWasRejected(bipedalFootstepPlannerNode, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_WIGGLE_INSIDE);

         //TODO: Possibly have different node costs depending on how firm on ground they are.
         if (parameters.getRejectIfCannotFullyWiggleInside())
         {
            return null;
         }

         else
         {
            return snapTransform;
         }
      }

      //      System.out.println("wiggleTransformLocalToLocal = \n" + wiggleTransformLocalToLocal);

      //      wiggleTransform = new RigidBodyTransform();
      //      wiggleTransform.setTranslation(0.2, 0.0, 0.0);

      Point3D wiggleTranslation = new Point3D();
      wiggleTransformLocalToLocal.transform(wiggleTranslation);
      Vector3D wiggleVector = new Vector3D(wiggleTranslation);
      if (wiggleVector.length() > parameters.getMaximumXYWiggleDistance())
      {
         wiggleVector.scale(parameters.getMaximumXYWiggleDistance() / wiggleVector.length());
      }

      Vector3D rotationEuler = new Vector3D();
      wiggleTransformLocalToLocal.getRotationEuler(rotationEuler);
      double yaw = rotationEuler.getZ();
      yaw = MathTools.clamp(yaw, parameters.getMaximumYawWiggle());

      rotationEuler.setZ(yaw);
      wiggleTransformLocalToLocal.setRotationEulerAndZeroTranslation(rotationEuler);
      wiggleTransformLocalToLocal.setTranslation(wiggleVector);

      //      System.out.println("Limited wiggleTransformLocalToLocal = \n" + wiggleTransformLocalToLocal);

      RigidBodyTransform wiggleTransformWorldToWorld = new RigidBodyTransform();
      RigidBodyTransform transformOne = new RigidBodyTransform();
      planarRegionToPack.getTransformToWorld(transformOne);
      RigidBodyTransform transformTwo = new RigidBodyTransform(transformOne);
      transformTwo.invert();

      wiggleTransformWorldToWorld.set(transformOne);
      wiggleTransformWorldToWorld.multiply(wiggleTransformLocalToLocal);
      wiggleTransformWorldToWorld.set(wiggleTransformWorldToWorld);
      wiggleTransformWorldToWorld.multiply(transformTwo);

      //      System.out.println("wiggleTransformWorldToWorld = \n" + wiggleTransformWorldToWorld);

      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform(wiggleTransformWorldToWorld);
      snapAndWiggleTransform.multiply(snapTransform);

      // Ensure polygon will be completely above the planarRegions with this snap and wiggle:
      ConvexPolygon2D checkFootPolygonInWorld = new ConvexPolygon2D(currentFootPolygon);
      checkFootPolygonInWorld.applyTransformAndProjectToXYPlane(snapAndWiggleTransform);

      List<PlanarRegion> planarRegionsIntersectingSnappedAndWiggledPolygon = planarRegionsList.findPlanarRegionsIntersectingPolygon(checkFootPolygonInWorld);

      ArrayList<ConvexPolygon2D> intersectionsInPlaneFrameToPack = new ArrayList<>();
      RigidBodyTransform transformToWorldFromIntersectingPlanarRegion = new RigidBodyTransform();

      if (planarRegionsIntersectingSnappedAndWiggledPolygon != null)
      {
         for (PlanarRegion planarRegionIntersectingSnappedAndWiggledPolygon : planarRegionsIntersectingSnappedAndWiggledPolygon)
         {
            planarRegionIntersectingSnappedAndWiggledPolygon.getTransformToWorld(transformToWorldFromIntersectingPlanarRegion);
            intersectionsInPlaneFrameToPack.clear();
            planarRegionIntersectingSnappedAndWiggledPolygon.getPolygonIntersectionsWhenProjectedVertically(checkFootPolygonInWorld,
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

                  double planeZGivenXY = planarRegionToPack.getPlaneZGivenXY(vertex3dInWorld.getX(), vertex3dInWorld.getY());

                  double zPenetration = vertex3dInWorld.getZ() - planeZGivenXY;
                  //               System.out.println("zPenetration = " + zPenetration);

                  if (zPenetration > parameters.getMaximumZPenetrationOnValleyRegions())
                  {
                     notifyListenerNodeUnderConsiderationWasRejected(bipedalFootstepPlannerNode,
                                                                     BipedalFootstepPlannerNodeRejectionReason.TOO_MUCH_PENETRATION_AFTER_WIGGLE);
                     return null;
                  }
               }
            }
         }
      }

      return snapAndWiggleTransform;
   }

   private boolean isTransformZUp(RigidBodyTransform soleTransformBeforeSnap)
   {
      return Math.abs(soleTransformBeforeSnap.getM22() - 1.0) < 1e-4;
   }

   private void notifyListenerNodeUnderConsiderationWasRejected(FootstepNode nodeToExpand, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (listener != null)
      {
         listener.nodeUnderConsiderationWasRejected(nodeToExpand, reason);
      }
   }
}
