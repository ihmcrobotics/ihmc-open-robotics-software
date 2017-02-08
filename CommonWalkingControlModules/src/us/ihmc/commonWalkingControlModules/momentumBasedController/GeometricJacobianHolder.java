package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

/*
 * Change the strategy to be less expensive in terms of computation.
 * The method compute() should be replaced a method reset() which should flip a boolean somehow linked to each jacobian indicating that they need to be updated.
 * When a getter is called, call GeometricJacobian.compute() only if needed.
 * That should allow the user to create as many jacobian as needed and update only the ones that are actually being used.
 */
public class GeometricJacobianHolder
{
   public static final long NULL_JACOBIAN_ID = NameBasedHashCodeTools.NULL_HASHCODE;

   private final TLongObjectHashMap<GeometricJacobian> nameBasedHashCodeToJacobianMap = new TLongObjectHashMap<GeometricJacobian>();
   private final List<GeometricJacobian> geometricJacobians = new ArrayList<GeometricJacobian>();
   private final InverseDynamicsJoint[] temporaryToStoreJointPath = new InverseDynamicsJoint[30];

   public void compute()
   {
      for (int i = 0; i < geometricJacobians.size(); i++)
      {
         geometricJacobians.get(i).compute();
      }
   }

   /**
    * Find or create a Jacobian and register it in the MomentumBasedController.
    * It returns an jacobianId with which it is possible to find the Jacobian later with the method getJacobian(int jacobianId).
    * @param ancestor
    * @param descendant
    * @param jacobianFrame
    * @return
    */
   public long getOrCreateGeometricJacobian(RigidBody ancestor, RigidBody descendant, ReferenceFrame jacobianFrame)
   {
      int numberOfJoints = ScrewTools.createJointPath(temporaryToStoreJointPath, ancestor, descendant);
      return getOrCreateGeometricJacobian(temporaryToStoreJointPath, numberOfJoints, jacobianFrame);
   }

   /**
    * Find or create a Jacobian and register it in the MomentumBasedController.
    * It returns an jacobianId with which it is possible to find the Jacobian later with the method getJacobian(int jacobianId).
    * If the array of joints is empty, it returns -1.
    * @param joints
    * @param jacobianFrame
    * @return
    */
   public long getOrCreateGeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame)
   {
      return getOrCreateGeometricJacobian(joints, joints.length, jacobianFrame);
   }

   private long getOrCreateGeometricJacobian(InverseDynamicsJoint[] joints, int numberOfJointsToConsider, ReferenceFrame jacobianFrame)
   {
      if (joints == null || numberOfJointsToConsider == 0)
         return NULL_JACOBIAN_ID;
      
      // The mapping assumes the frame do not change.
      // On top of that, this class makes the different modules use the same instances of each Jacobian, so it would not be good if one module changes the frame of a Jacobian shared with another module. 
      boolean allowChangeFrame = false;

      long jacobianId = ScrewTools.computeGeometricJacobianNameBasedHashCode(joints, 0, numberOfJointsToConsider - 1, jacobianFrame, allowChangeFrame);
      GeometricJacobian jacobian = getJacobian(jacobianId);

      if (jacobian == null)
      {
         if (joints.length == numberOfJointsToConsider)
         {
            jacobian = new GeometricJacobian(joints, jacobianFrame, allowChangeFrame);
         }
         else
         {
            InverseDynamicsJoint[] jointsForNewJacobian = new InverseDynamicsJoint[numberOfJointsToConsider];
            System.arraycopy(joints, 0, jointsForNewJacobian, 0, numberOfJointsToConsider);
            jacobian = new GeometricJacobian(jointsForNewJacobian, jacobianFrame, allowChangeFrame);
         }
         jacobian.compute(); // Compute in case you need it right away
         geometricJacobians.add(jacobian);
         nameBasedHashCodeToJacobianMap.put(jacobian.nameBasedHashCode(), jacobian);
      }

      return jacobian.nameBasedHashCode();
   }

   /**
    * Return a jacobian previously created with the getOrCreate method using a jacobianId.
    * @param jacobianId
    * @return
    */
   public GeometricJacobian getJacobian(long jacobianId)
   {
      if (jacobianId == NULL_JACOBIAN_ID)
         return null;
      else
         return nameBasedHashCodeToJacobianMap.get(jacobianId);
   }
}
