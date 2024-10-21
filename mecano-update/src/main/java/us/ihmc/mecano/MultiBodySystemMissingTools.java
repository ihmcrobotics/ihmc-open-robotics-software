package us.ihmc.mecano;

import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

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

   public static void copyOneDoFJointsConfiguration(JointReadOnly[] source, JointBasics[] destinationToPack)
   {
      OneDoFJointReadOnly[] oneDofJoints1 = MultiBodySystemTools.filterJoints(source, OneDoFJointReadOnly.class);
      OneDoFJointBasics[] oneDofJoints2 = MultiBodySystemTools.filterJoints(destinationToPack, OneDoFJointBasics.class);

      if (oneDofJoints1.length != oneDofJoints2.length)
      {
         throw new IllegalArgumentException("The lists of joints must be the same length. %d != %d".formatted(oneDofJoints1.length,
                                                                                                              oneDofJoints2.length));
      }

      for (int i = 0; i < oneDofJoints1.length; i++)
      {
         oneDofJoints2[i].setJointConfiguration(oneDofJoints1[i]);
      }
   }
}
