package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepNodeSnappingTools
{
   /**
    * Computes the convex hull of the intersections of footPolygonInWorldFrame when snapped to planarRegion using
    * snapTransform
    *
    * @param planarRegion
    * @param footPolygon foot polygon in world frame
    * @param snapTransform
    * @return intersection polygon in region frame
    */
   public static ConvexPolygon2D getConvexHullOfPolygonIntersections(PlanarRegion planarRegion, ConvexPolygon2D footPolygon,
                                                                     RigidBodyTransform snapTransform)
   {
      ArrayList<ConvexPolygon2D> intersections = new ArrayList<>();
      ConvexPolygon2D footPolygonInPlaneFrame = new ConvexPolygon2D();

      RigidBodyTransform inverseSnapTransform = new RigidBodyTransform(snapTransform);
      inverseSnapTransform.invert();

      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = footPolygon.getVertex(i);
         Vector4D transformPoint = new Vector4D(vertex.getX(), vertex.getY(), 0.0, 1.0);
         snapTransform.transform(transformPoint);
         transformPoint.setZ(0.0);
         footPolygonInPlaneFrame.addVertex(transformPoint.getX() + 1e-10, transformPoint.getY() + 1e-10);
      }
      footPolygonInPlaneFrame.update();

      planarRegion.getPolygonIntersectionsWhenProjectedVertically(footPolygonInPlaneFrame, intersections);
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
    * Transforms footPolygonInRegionFrame from planarRegion frame to the snapped footstep node frame
    *
    * @param planarRegion
    * @param footstepNode
    * @param snapTransform
    * @param footPolygonInRegionFrame
    */
   public static void changeFromPlanarRegionToSoleFrame(PlanarRegion planarRegion, FootstepNode footstepNode, RigidBodyTransform snapTransform,
                                                        ConvexPolygon2D footPolygonInRegionFrame)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(regionToWorld);

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransform(footstepNode, snapTransform, soleTransform);

      RigidBodyTransform regionToSole = new RigidBodyTransform();
      regionToSole.setAndInvert(soleTransform);
      regionToSole.multiply(regionToWorld);

      footPolygonInRegionFrame.applyTransformAndProjectToXYPlane(regionToSole);
   }

   /**
    * Computes the snap transform which snaps the given node to the given pose
    *
    * @param node
    * @param footstepPose
    * @return
    */
   public static RigidBodyTransform computeSnapTransform(FootstepNode node, Pose3DReadOnly footstepPose)
   {
      RigidBodyTransform snapTransform = new RigidBodyTransform();
      RigidBodyTransform stepTransform = new RigidBodyTransform();
      footstepPose.get(stepTransform);

      FootstepNodeTools.getNodeTransform(node, snapTransform);
      snapTransform.preMultiplyInvertThis(stepTransform);

      return snapTransform;
   }

   /**
    * Adds a planar region near midFootZUp if there are no planar regions in the vicinity
    */
   public static void constructGroundPlaneAroundFeet(PlanarRegionsList planarRegionsList, FootstepNode stanceFootNode, RigidBodyTransform snapTransform, double idealStepWidth, double searchWidth,
                                                             double minSearchLength, double maxSearchLength)
   {
      Point2D midFootPoint = stanceFootNode.getOrComputeMidFootPoint(idealStepWidth);
      double yaw = stanceFootNode.getYaw();

      RigidBodyTransform footstepPose = new RigidBodyTransform();
      footstepPose.setRotationYawAndZeroTranslation(yaw);
      footstepPose.setTranslationX(midFootPoint.getX());
      footstepPose.setTranslationY(midFootPoint.getY());
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
