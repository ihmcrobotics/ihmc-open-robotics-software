package us.ihmc.robotics.screwTheory;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

public class TotalMassCalculator
{
   public static double computeSubTreeMass(RigidBodyReadOnly rootBody)
   {
      SpatialInertiaReadOnly inertia = rootBody.getInertia();
      double ret = inertia == null ? 0.0 : inertia.getMass();

      for (JointReadOnly childJoint : rootBody.getChildrenJoints())
      {
         ret += computeSubTreeMass(childJoint.getSuccessor());
      }

      return ret;
   }

   public static double computeMass(RigidBodyReadOnly[] rigidBodies)
   {
      double ret = 0.0;
      for (int i = 0; i < rigidBodies.length; i++)
      {
         ret += rigidBodies[i].getInertia().getMass();
      }
      return ret;
   }

   public static double computeMass(Iterable<? extends RigidBodyReadOnly> rigidBodies)
   {
      double ret = 0.0;
      for (RigidBodyReadOnly rigidBody : rigidBodies)
      {
         ret += rigidBody.getInertia().getMass();
      }
      return ret;
   }

   public static double computeMass(List<? extends RigidBodyReadOnly> rigidBodies)
   {
      double ret = 0.0;
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         ret += rigidBodies.get(i).getInertia().getMass();
      }
      return ret;
   }
}
