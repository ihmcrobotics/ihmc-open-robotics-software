package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class BipedalFootstepPlannerNodeUtils
{
   public static RigidBodyTransform shiftInSoleFrame(Vector2D shiftVector, RigidBodyTransform soleTransform)
   {
      RigidBodyTransform shiftTransform = new RigidBodyTransform();
      shiftTransform.setTranslation(new Vector3D(shiftVector.getX(), shiftVector.getY(), 0.0));
      soleTransform.multiply(shiftTransform);
      return soleTransform;
   }

   public static void getSoleTransform(FootstepNode node, RigidBodyTransform transformToPack)
   {
      double soleYaw = node.getYaw();
      Point3D solePosition = getSolePosition(node);
      transformToPack.setRotationYawAndZeroTranslation(soleYaw);
      transformToPack.setTranslation(solePosition);
   }

   public static RigidBodyTransform getSnappedSoleTransform(FootstepNode node, RigidBodyTransform snapTransform)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      getSoleTransform(node, soleTransform);
      snapTransform.transform(soleTransform);
      return soleTransform;
   }

   public static Point3D getSolePosition(FootstepNode node)
   {
      return new Point3D(node.getX(), node.getY(), 0.0);
   }
}
