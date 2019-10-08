package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.AngleTools;

public class FootstepNodeTools
{
   public static RigidBodyTransform shiftInSoleFrame(Vector2D shiftVector, RigidBodyTransform soleTransform)
   {
      RigidBodyTransform shiftTransform = new RigidBodyTransform();
      shiftTransform.setTranslation(new Vector3D(shiftVector.getX(), shiftVector.getY(), 0.0));
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
      double soleYaw = node.getYaw();
      Point3D solePosition = new Point3D(node.getX(), node.getY(), 0.0);
      nodeToWorldTransformToPack.setRotationYawAndZeroTranslation(soleYaw);
      nodeToWorldTransformToPack.setTranslation(solePosition);
   }

   /**
    * Computes a node-to-world RigidBodyTransform which transforms from node frame to world frame
    * by applying snapTransform to the given node's transform
    *
    * @param node
    * @param snapTransform pre-snap to post-snap transform
    * @param transformToPack
    */
   public static void getSnappedNodeTransform(FootstepNode node, RigidBodyTransform snapTransform, RigidBodyTransform transformToPack)
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

   /**
    * Computes the foot polygon in world frame that corresponds to the give footstep node
    *
    * @param node
    * @param footPolygonInSoleFrame
    * @param footPolygonToPack
    */
   public static void getFootPolygon(FootstepNode node, ConvexPolygon2D footPolygonInSoleFrame, ConvexPolygon2D footPolygonToPack)
   {
      footPolygonToPack.set(footPolygonInSoleFrame);

      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(node, nodeTransform);

      footPolygonToPack.applyTransform(nodeTransform);
   }

   public static LatticeNode interpolate(LatticeNode nodeA, LatticeNode nodeB, double alpha)
   {
      double x = EuclidCoreTools.interpolate(nodeA.getX(), nodeB.getX(), alpha);
      double y = EuclidCoreTools.interpolate(nodeA.getY(), nodeB.getY(), alpha);
      double yaw = AngleTools.interpolateAngle(nodeA.getYaw(), nodeB.getYaw(), alpha);
      return new LatticeNode(x, y, yaw);
   }
}
