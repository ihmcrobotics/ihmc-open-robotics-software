package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

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
    * by applying {@snapTransform} to the given node's transform
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
}
