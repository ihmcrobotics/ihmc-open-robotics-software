package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FootstepSnappingTools
{
   /**
    * Computes the snapped polygon using the given snap transform
    */
   public static ConvexPolygon2D computeTransformedPolygon(ConvexPolygon2DReadOnly initialPolygon, RigidBodyTransformReadOnly transform)
   {
      ConvexPolygon2D footPolygonInRegionFrame = new ConvexPolygon2D();
      Vector4D transformedVertex = new Vector4D();

      for (int i = 0; i < initialPolygon.getNumberOfVertices(); i++)
      {
         transformPolygonVertex(initialPolygon.getVertex(i), transformedVertex, transform);
         footPolygonInRegionFrame.addVertex(transformedVertex.getX() + 1e-10, transformedVertex.getY() + 1e-10);
      }

      footPolygonInRegionFrame.update();
      return footPolygonInRegionFrame;
   }

   public static void transformPolygonVertex(Point2DReadOnly vertex, Vector4D transformedVertexToPack, RigidBodyTransformReadOnly transform)
   {
      transformedVertexToPack.set(vertex.getX(), vertex.getY(), 0.0, 1.0);
      transform.transform(transformedVertexToPack);
   }

   /**
    * Computes the intersection of the given polygon with the region's convex polygons
    */
   public static ConvexPolygon2D computeRegionIntersection(PlanarRegion planarRegion, ConvexPolygon2DReadOnly polygonInPlaneFrame)
   {
      ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
      planarRegion.getPolygonIntersectionsWhenProjectedVertically(polygonInPlaneFrame, intersections);
      return getConvexHull(intersections);
   }

   /**
    * Computes the convex hull of the given list of intersections. Returns an empty ConvexPolygon2D
    * if the list is empty
    *
    * @param intersections
    * @return
    */
   private static ConvexPolygon2D getConvexHull(ArrayList<ConvexPolygon2D> intersections)
   {
      ConvexPolygon2D combinedFootholdIntersection = new ConvexPolygon2D();
      for (int i = 0; i < intersections.size(); i++)
      {
         combinedFootholdIntersection.addVertices(intersections.get(i));
      }

      combinedFootholdIntersection.update();
      return combinedFootholdIntersection;
   }

   /**
    * Transforms footPolygonInRegionFrame from planarRegion frame to the snapped footstep frame
    *
    * @param planarRegion
    * @param footstep
    * @param snapTransform
    * @param footPolygonInRegionFrame
    */
   public static void changeFromPlanarRegionToSoleFrame(PlanarRegion planarRegion, DiscreteFootstep footstep, RigidBodyTransform snapTransform,
                                                        ConvexPolygon2D footPolygonInRegionFrame)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(regionToWorld);

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      DiscreteFootstepTools.getSnappedStepTransform(footstep, snapTransform, soleTransform);

      RigidBodyTransform regionToSole = new RigidBodyTransform();
      regionToSole.setAndInvert(soleTransform);
      regionToSole.multiply(regionToWorld);

      footPolygonInRegionFrame.applyTransform(regionToSole, false);
   }

   /**
    * Computes the snap transform which snaps the given footstep to the given pose
    *
    * @param footstep
    * @param footstepPose
    * @return
    */
   public static RigidBodyTransform computeSnapTransform(DiscreteFootstep footstep, Pose3DReadOnly footstepPose)
   {
      RigidBodyTransform snapTransform = new RigidBodyTransform();
      RigidBodyTransform stepTransform = new RigidBodyTransform();
      footstepPose.get(stepTransform);

      DiscreteFootstepTools.getStepTransform(footstep, snapTransform);
      snapTransform.preMultiplyInvertThis(stepTransform);

      return snapTransform;
   }

   /**
    * Adds a planar region near midFootZUp if there are no planar regions in the vicinity
    */
   public static void constructGroundPlaneAroundFeet(PlanarRegionsList planarRegionsList, DiscreteFootstep stanceFootstep, RigidBodyTransform snapTransform, double idealStepWidth, double searchWidth,
                                                     double minSearchLength, double maxSearchLength)
   {
      Point2D midFootPoint = stanceFootstep.getOrComputeMidFootPoint(idealStepWidth);
      double yaw = stanceFootstep.getYaw();

      RigidBodyTransform footstepPose = new RigidBodyTransform();
      footstepPose.setRotationYawAndZeroTranslation(yaw);
      footstepPose.getTranslation().setX(midFootPoint.getX());
      footstepPose.getTranslation().setY(midFootPoint.getY());
      snapTransform.transform(footstepPose);

      int numberOfChecks = 5;
      double lengthIncrement = (maxSearchLength - minSearchLength) / ((double) numberOfChecks - 1);

      ArrayList<PlanarRegion> intersectingRegions = new ArrayList<>();

      for (int i = 0; i < numberOfChecks; i++)
      {
         double boxLength = minSearchLength + i * lengthIncrement;
         LineSegment2D lineSegment = new LineSegment2D(boxLength, -0.5 * searchWidth, boxLength, 0.5 * searchWidth);
         lineSegment.applyTransform(footstepPose);

         intersectingRegions.clear();
         planarRegionsList.findPlanarRegionsIntersectingLineSegment(lineSegment, intersectingRegions);

         if(intersectingRegions.size() > 0)
         {
            if(i == 0)
               return;
            planarRegionsList.addPlanarRegion(constructPlanarRegion(footstepPose, searchWidth, boxLength - lengthIncrement));
         }
      }

      planarRegionsList.addPlanarRegion(constructPlanarRegion(footstepPose, searchWidth, maxSearchLength));
   }

   private static PlanarRegion constructPlanarRegion(RigidBodyTransform transformToWorld, double width, double length)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.0, -0.5 * length);
      polygon.addVertex(0.0, 0.5 * length);
      polygon.addVertex(width, -0.5 * length);
      polygon.addVertex(width, 0.5 * length);
      polygon.update();

      return new PlanarRegion(transformToWorld, polygon);
   }
}
