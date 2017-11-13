package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;

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
   public static RigidBodyTransform computeSnapTransform(FootstepNode node, Pose3D footstepPose)
   {
      RigidBodyTransform snapTransform = new RigidBodyTransform();
      RigidBodyTransform stepTransform = new RigidBodyTransform();
      footstepPose.get(stepTransform);

      FootstepNodeTools.getNodeTransform(node, snapTransform);
      snapTransform.preMultiplyInvertThis(stepTransform);

      return snapTransform;
   }
}
