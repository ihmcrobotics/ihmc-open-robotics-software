package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCentroidalMomentumMatrixCalculator
{
   private final DynamicallyConsistentNullspaceCalculator dynamicallyConsistentNullspaceCalculator;
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final DMatrixRMaj selectionMatrix;
   private final DMatrixRMaj temp = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj constrainedCentroidalMomentumMatrix = new DMatrixRMaj(1, 1);

   public ConstrainedCentroidalMomentumMatrixCalculator(FloatingJointBasics rootJoint, ReferenceFrame centerOfMassFrame,
                                                        DMatrixRMaj selectionMatrix)
   {
      this.dynamicallyConsistentNullspaceCalculator = new OriginalDynamicallyConsistentNullspaceCalculator(rootJoint,
            true);
      this.centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootJoint.getSuccessor(), centerOfMassFrame);
      this.selectionMatrix = selectionMatrix;
   }

   public void reset()
   {
      dynamicallyConsistentNullspaceCalculator.reset();
   }

   public void addConstraint(RigidBodyBasics body, DMatrixRMaj selectionMatrix)
   {
      dynamicallyConsistentNullspaceCalculator.addConstraint(body, selectionMatrix);
   }

   public void addActuatedJoint(JointBasics joint)
   {
      dynamicallyConsistentNullspaceCalculator.addActuatedJoint(joint);
   }

   public void compute()
   {
      dynamicallyConsistentNullspaceCalculator.compute();
      centroidalMomentumCalculator.reset();
      DMatrixRMaj centroidalMomentumMatrix = this.centroidalMomentumCalculator.getCentroidalMomentumMatrix();
      DMatrixRMaj sNsBar = dynamicallyConsistentNullspaceCalculator.getSNsBar();

      temp.reshape(centroidalMomentumMatrix.getNumRows(), sNsBar.getNumCols());
      CommonOps_DDRM.mult(centroidalMomentumMatrix, sNsBar, temp);

      constrainedCentroidalMomentumMatrix.reshape(selectionMatrix.getNumRows(), temp.getNumCols());
      CommonOps_DDRM.mult(selectionMatrix, temp, constrainedCentroidalMomentumMatrix);
   }

   public DMatrixRMaj getConstrainedCentroidalMomentumMatrix()
   {
      return constrainedCentroidalMomentumMatrix;
   }

   public DMatrixRMaj getCentroidalMomentumMatrix()
   {
      return centroidalMomentumCalculator.getCentroidalMomentumMatrix();
   }
}
