package us.ihmc.robotics;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

public class MultiBodySystemMissingTools
{
   /**
    * Computes the mass of all the links contained in the subtree of {@param rootBody}
    * @param rootBody
    * @return subtree mass
    */
   public static double computeSubTreeMass(RigidBodyReadOnly rootBody)
   {
      SpatialInertiaReadOnly inertia = rootBody.getInertia();
      double ret = inertia == null ? 0.0 : inertia.getMass();

      for (int i = 0; i < rootBody.getChildrenJoints().size(); i++)
      {
         ret += computeSubTreeMass(rootBody.getChildrenJoints().get(i).getSuccessor());
      }

      return ret;
   }
}
