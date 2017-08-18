package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.RotationTools;

public class BipedalFootstepPlannerNodeUtils
{
   public static void shiftInSoleFrame(Vector2D shiftVector, BipedalFootstepPlannerNode node)
   {
      RigidBodyTransform shiftTransform = new RigidBodyTransform();
      shiftTransform.setTranslation(new Vector3D(shiftVector.getX(), shiftVector.getY(), 0.0));

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      node.getSoleTransform(soleTransform);

      soleTransform.multiply(shiftTransform);
      node.setSoleTransform(soleTransform);
   }

   public static void transformSoleTransformWithSnapTransformFromZeroZ(RigidBodyTransform snapTransform, BipedalFootstepPlannerNode node)
   {
      // Ignore the z since the snap transform snapped from z = 0. Keep everything else.
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      node.getSoleTransform(soleTransform);

      soleTransform.setTranslationZ(0.0);
      soleTransform.preMultiply(snapTransform);

      node.setSoleTransform(soleTransform);
   }

   public static Point3D getSolePosition(BipedalFootstepPlannerNode node)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      node.getSoleTransform(soleTransform);

      Point3D currentSolePosition = new Point3D();
      soleTransform.transform(currentSolePosition);
      return currentSolePosition;
   }

   public static double getSoleYaw(BipedalFootstepPlannerNode node)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      node.getSoleTransform(soleTransform);

      Vector3D eulerAngles = new Vector3D();
      soleTransform.getRotationEuler(eulerAngles);
      return eulerAngles.getZ();
   }

   public static void removePitchAndRoll(BipedalFootstepPlannerNode node)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      node.getSoleTransform(soleTransform);
      RotationTools.removePitchAndRollFromTransform(soleTransform);
      node.setSoleTransform(soleTransform);
   }

   public static double getCostFromStartToNode(BipedalFootstepPlannerNode node)
   {
      if (node.getParentNode() == null)
         return node.getSingleStepCost();
      return node.getSingleStepCost() + getCostFromStartToNode(node.getParentNode());
   }
}
