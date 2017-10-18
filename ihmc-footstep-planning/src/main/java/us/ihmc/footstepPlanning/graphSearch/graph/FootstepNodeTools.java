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
    * Computes a RigidBodyTransform from the node's x, y and yaw. This transform
    * will always have no z translation, pitch and roll.
    * @param node
    * @param transformToPack
    */
   public static void getNodeTransform(FootstepNode node, RigidBodyTransform transformToPack)
   {
      double soleYaw = node.getYaw();
      Point3D solePosition = new Point3D(node.getX(), node.getY(), 0.0);
      transformToPack.setRotationYawAndZeroTranslation(soleYaw);
      transformToPack.setTranslation(solePosition);
   }

   /**
    * Computes a RigidBodyTransform by applying {@snapTransform} to the given node's transform
    *
    * @param node
    * @param snapTransform
    * @param transformToPack
    */
   public static void getSnappedNodeTransform(FootstepNode node, RigidBodyTransform snapTransform, RigidBodyTransform transformToPack)
   {
      getNodeTransform(node, transformToPack);
      snapTransform.transform(transformToPack);
   }
}
