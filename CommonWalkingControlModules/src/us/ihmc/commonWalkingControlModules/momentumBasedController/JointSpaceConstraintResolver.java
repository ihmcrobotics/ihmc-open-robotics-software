package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.ScrewTools;

public class JointSpaceConstraintResolver
{
   private final InverseDynamicsJoint[] jointsInOrder;

   private final DenseMatrix64F aJointSpace;
   private final DenseMatrix64F aJointSpaceVdotJointSpace;

   public JointSpaceConstraintResolver(InverseDynamicsJoint[] jointsInOrder)
   {
      this.jointsInOrder = jointsInOrder;

      int size = Momentum.SIZE;
      this.aJointSpaceVdotJointSpace = new DenseMatrix64F(size, 1);
      this.aJointSpace = new DenseMatrix64F(size, size);    // reshaped
   }

   public void handleJointSpaceAcceleration(DenseMatrix64F b, DenseMatrix64F centroidalMomentumMatrix, InverseDynamicsJoint joint,
           DenseMatrix64F jointSpaceAcceleration)
   {
      int[] jointSpaceColumnIndices = ScrewTools.computeIndicesForJoint(jointsInOrder, joint);
      aJointSpace.reshape(aJointSpace.getNumRows(), jointSpaceColumnIndices.length);
      MatrixTools.extractColumns(centroidalMomentumMatrix, aJointSpace, jointSpaceColumnIndices);

      CommonOps.mult(aJointSpace, jointSpaceAcceleration, aJointSpaceVdotJointSpace);
      CommonOps.subEquals(b, aJointSpaceVdotJointSpace);
   }

   public void solveAndSetJointspaceAccelerations(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations)
   {
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         joint.setDesiredAcceleration(jointSpaceAccelerations.get(joint), 0);
      }
   }
}
