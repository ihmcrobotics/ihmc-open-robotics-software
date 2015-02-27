package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;

public class GeometricJacobianHolder
{
   private final List<GeometricJacobian> robotJacobians = new ArrayList<GeometricJacobian>();
   
   public void compute()
   {
      for (int i = 0; i < robotJacobians.size(); i++)
      {
         robotJacobians.get(i).compute();
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
   public int getOrCreateGeometricJacobian(RigidBody ancestor, RigidBody descendant, ReferenceFrame jacobianFrame)
   {
      return getOrCreateGeometricJacobian(ScrewTools.createJointPath(ancestor, descendant), jacobianFrame);
   }
   
   /**
    * Find or create a Jacobian and register it in the MomentumBasedController.
    * It returns an jacobianId with which it is possible to find the Jacobian later with the method getJacobian(int jacobianId).
    * If the array of joints is empty, it returns -1.
    * @param joints
    * @param jacobianFrame
    * @return
    */
   public int getOrCreateGeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame)
   {
      if (joints == null || joints.length == 0)
         return -1;

      for (int i = 0; i < robotJacobians.size(); i++)
      {
         GeometricJacobian jacobian = robotJacobians.get(i);
         InverseDynamicsJoint[] existingJacobianJoints = jacobian.getJointsInOrder();
         boolean sameNumberOfJoints = joints.length == existingJacobianJoints.length;
         boolean areExpressedFrameTheSame = jacobianFrame == jacobian.getJacobianFrame();
         
         if (sameNumberOfJoints && areExpressedFrameTheSame)
         {
            boolean allJointsAreTheSame = true;
            // The joint arrays are considered to be in the same order
            for (int j = 0; j < existingJacobianJoints.length; j++)
            {
               boolean jointsAreTheSame = joints[j] == existingJacobianJoints[j];
               if (!jointsAreTheSame)
               {
                  allJointsAreTheSame = false;
                  break;
               }
            }
            if (allJointsAreTheSame)
               return i;
         }
      }

      GeometricJacobian newJacobian = new GeometricJacobian(joints, jacobianFrame);
      newJacobian.compute(); // Compute in case you need it right away
      int jacobianId = robotJacobians.size();
      robotJacobians.add(newJacobian);
      return jacobianId;
   }
   
   /**
    * Return a jacobian previously created with the getOrCreate method using a jacobianId.
    * @param jacobianId
    * @return
    */
   public GeometricJacobian getJacobian(int jacobianId)
   {
      if (jacobianId >= robotJacobians.size() || jacobianId < 0)
         return null;
      return robotJacobians.get(jacobianId);
   }
}
