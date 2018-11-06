package us.ihmc.robotics.screwTheory;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

public class TotalMassCalculator
{
   public static double computeSubTreeMass(RigidBodyBasics rootBody)
   {
      SpatialInertiaBasics inertia = rootBody.getInertia();
      double ret = inertia == null ? 0.0 : inertia.getMass();

      for (JointBasics childJoint : rootBody.getChildrenJoints())
      {
         ret += computeSubTreeMass(childJoint.getSuccessor());
      }

      return ret;
   }

   public static double computeMass(RigidBodyBasics[] rigidBodies)
   {
      double ret = 0.0;
      for (int i = 0; i < rigidBodies.length; i++)
      {
         ret += rigidBodies[i].getInertia().getMass();
      }
      return ret;
   }

   public static double computeMass(Iterable<RigidBodyBasics> rigidBodies)
   {
      double ret = 0.0;
      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         ret += rigidBody.getInertia().getMass();
      }
      return ret;
   }

   public static double computeMass(List<RigidBodyBasics> rigidBodies)
   {
      double ret = 0.0;
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         ret += rigidBodies.get(i).getInertia().getMass();
      }
      return ret;
   }
}
