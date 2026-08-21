package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class InertiaAboutJointCalculator
{
   public static void computeForChildren(RigidBodyReadOnly body)
   {
      for (JointReadOnly joint : body.getChildrenJoints())
      {
         computeForChildren(joint.getSuccessor());

         if (joint instanceof OneDoFJointReadOnly oneDof)
            computeAndPrintJointInertia(oneDof);
      }
   }

   private static void computeAndPrintJointInertia(OneDoFJointReadOnly jointBasics)
   {
      SpatialInertia inertia = MultiBodySystemTools.computeSubtreeInertia(jointBasics);
      DMatrixRMaj inertiaMatrix = new DMatrixRMaj(3, 3);
      DMatrixRMaj rotationVector = new DMatrixRMaj(3, 1);
      inertia.getMomentOfInertia().get(inertiaMatrix);
      jointBasics.getJointAxis().get(rotationVector);
      DMatrixRMaj rightSide = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(inertiaMatrix, rotationVector, rightSide);
      DMatrixRMaj principal = new DMatrixRMaj(1, 1);
      CommonOps_DDRM.multTransA(rotationVector, rightSide, principal);

      LogTools.info("Inertia about joint " + jointBasics.getName() + " is " + principal.get(0, 0) + ", and mass is " + inertia.getMass());
   }

}
