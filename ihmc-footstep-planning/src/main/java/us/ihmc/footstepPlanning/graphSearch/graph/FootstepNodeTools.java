package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepNodeTools
{
   public static RigidBodyTransform shiftInSoleFrame(Vector2D shiftVector, RigidBodyTransform soleTransform)
   {
      RigidBodyTransform shiftTransform = new RigidBodyTransform();
      shiftTransform.getTranslation().set(new Vector3D(shiftVector.getX(), shiftVector.getY(), 0.0));
      soleTransform.multiply(shiftTransform);
      return soleTransform;
   }

   /**
    * Computes a node-to-world RigidBodyTransform from the node's x, y and yaw. This transform
    * will always have no z translation, pitch and roll.
    * @param node
    * @param nodeToWorldTransformToPack
    */
   public static void getNodeTransform(FootstepNode node, RigidBodyTransform nodeToWorldTransformToPack)
   {
      nodeToWorldTransformToPack.setRotationYawAndZeroTranslation(node.getYaw());
      nodeToWorldTransformToPack.getTranslation().set(node.getX(), node.getY(), 0.0);
   }

   /**
    * Computes a node-to-world RigidBodyTransform which transforms from node frame to world frame
    * by applying snapTransform to the given node's transform
    *
    * @param node
    * @param snapTransform pre-snap to post-snap transform
    * @param transformToPack
    */
   public static void getSnappedNodeTransform(FootstepNode node, RigidBodyTransformReadOnly snapTransform, RigidBodyTransform transformToPack)
   {
      getNodeTransform(node, transformToPack);
      snapTransform.transform(transformToPack);
   }

   public static Point3DReadOnly getNodePositionInWorld(FootstepNode node, RigidBodyTransform snapTransform)
   {
      Point3D nodeInWorld = new Point3D();
      getNodePositionInWorld(node, nodeInWorld, snapTransform);

      return nodeInWorld;
   }

   public static void getNodePositionInWorld(FootstepNode node, Point3DBasics nodeInWorld, RigidBodyTransform snapTransform)
   {
      nodeInWorld.set(node.getX(), node.getY(), 0.0);
      snapTransform.transform(nodeInWorld);
   }

   public static Pose3DReadOnly getNodePoseInWorld(FootstepNode node, RigidBodyTransform snapTransform)
   {
      Pose3D nodeInWorld = new Pose3D();
      getNodePoseInWorld(node, nodeInWorld, snapTransform);

      return nodeInWorld;
   }

   public static void getNodePoseInWorld(FootstepNode node, Pose3DBasics nodeInWorld, RigidBodyTransform snapTransform)
   {
      RigidBodyTransform snappedTransform = new RigidBodyTransform();
      getSnappedNodeTransform(node, snapTransform, snappedTransform);
      nodeInWorld.set(snappedTransform);
   }
   /**
    * Computes the foot polygon in world frame that corresponds to the give footstep node
    *
    * @param node
    * @param footPolygonInSoleFrame
    * @param footPolygonToPack
    */
   public static void getFootPolygon(FootstepNode node, ConvexPolygon2DReadOnly footPolygonInSoleFrame, ConvexPolygon2D footPolygonToPack)
   {
      footPolygonToPack.set(footPolygonInSoleFrame);

      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(node, nodeTransform);

      footPolygonToPack.applyTransform(nodeTransform);
   }

   public static double getSnappedNodeHeight(FootstepNode footstepNode, RigidBodyTransform snapTransform)
   {
      return snapTransform.getRotation().getM20() * footstepNode.getX() + snapTransform.getRotation().getM21() * footstepNode.getY() + snapTransform.getTranslationZ();
   }

   public static LatticeNode interpolate(LatticeNode nodeA, LatticeNode nodeB, double alpha)
   {
      double x = EuclidCoreTools.interpolate(nodeA.getX(), nodeB.getX(), alpha);
      double y = EuclidCoreTools.interpolate(nodeA.getY(), nodeB.getY(), alpha);
      double yaw = AngleTools.interpolateAngle(nodeA.getYaw(), nodeB.getYaw(), alpha);
      return new LatticeNode(x, y, yaw);
   }

   public static double computeDistanceBetweenFootPolygons(FootstepNode nodeA,
                                                           FootstepNode nodeB,
                                                           SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygonsInSoleFrame)
   {
      ConvexPolygon2D footPolygonA = new ConvexPolygon2D();
      ConvexPolygon2D footPolygonB = new ConvexPolygon2D();

      getFootPolygon(nodeA, footPolygonsInSoleFrame.get(nodeA.getRobotSide()), footPolygonA);
      getFootPolygon(nodeB, footPolygonsInSoleFrame.get(nodeB.getRobotSide()), footPolygonB);

      if (arePolygonsIntersecting(footPolygonA, footPolygonB))
      {
         return 0.0;
      }

      return distanceBetweenPolygons(footPolygonA, footPolygonB);
   }

   public static boolean arePolygonsIntersecting(ConvexPolygon2D polygonA, ConvexPolygon2D polygonB)
   {
      for (int i = 0; i < polygonA.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vA1 = polygonA.getVertex(i);
         Point2DReadOnly vA2 = polygonA.getNextVertex(i);

         // in case one polygon is completely contained in the other
         if (polygonB.isPointInside(vA1))
         {
            return true;
         }

         for (int j = 0; j < polygonB.getNumberOfVertices(); j++)
         {
            Point2DReadOnly vB1 = polygonB.getVertex(j);
            Point2DReadOnly vB2 = polygonB.getNextVertex(j);

            if (polygonA.isPointInside(vB1))
            {
               return true;
            }

            double vA1x = vA1.getX();
            double vA1y = vA1.getY();
            double vA2x = vA2.getX();
            double vA2y = vA2.getY();
            double vB1x = vB1.getX();
            double vB1y = vB1.getY();
            double vB2x = vB2.getX();
            double vB2y = vB2.getY();
            boolean intersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(vA1x, vA1y, vA2x, vA2y, vB1x, vB1y, vB2x, vB2y, null);
            if (intersection)
            {
               return true;
            }
         }
      }

      return false;
   }

   /**
    * Written assuming that the polygons aren't intersecting. This is brute force and probably less efficient
    * than {@link ConvexPolygonTools#computeMinimumDistancePoints}, but that method was seen to give bad results for polygons
    * intersecting by epsilon (didn't seem to be picked up by the method's initial intersecion check, and probably throws off the algorithm).
    */
   public static double distanceBetweenPolygons(ConvexPolygon2D polygonA, ConvexPolygon2D polygonB)
   {
      double minDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < polygonA.getNumberOfVertices(); i++)
      {
         for (int j = 0; j < polygonB.getNumberOfVertices(); j++)
         {
            double vA1x = polygonA.getVertex(i).getX();
            double vA1y = polygonA.getVertex(i).getY();
            double vA2x = polygonA.getNextVertex(i).getX();
            double vA2y = polygonA.getNextVertex(i).getY();
            double vB1x = polygonB.getVertex(j).getX();
            double vB1y = polygonB.getVertex(j).getY();
            double vB2x = polygonB.getNextVertex(j).getX();
            double vB2y = polygonB.getNextVertex(j).getY();
            double distance = EuclidGeometryTools.closestPoint2DsBetweenTwoLineSegment2Ds(vA1x, vA1y, vA2x, vA2y, vB1x, vB1y, vB2x, vB2y, null, null);
            minDistance = Math.min(minDistance, distance);
         }
      }

      return minDistance;
   }
}
