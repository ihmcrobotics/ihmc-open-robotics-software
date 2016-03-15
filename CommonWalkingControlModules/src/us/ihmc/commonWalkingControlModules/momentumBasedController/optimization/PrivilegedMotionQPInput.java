package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;

public class PrivilegedMotionQPInput
{
   public final DenseMatrix64F selectionMatrix;
   public final DenseMatrix64F privilegedJointspaceMotion;
   public final DenseMatrix64F weightMatrix;
   private final int numberOfDoFs;

   public PrivilegedMotionQPInput(int numberOfDoFs)
   {
      this.numberOfDoFs = numberOfDoFs;
      selectionMatrix = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      privilegedJointspaceMotion = new DenseMatrix64F(numberOfDoFs, 1);
      weightMatrix = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
   }

   public void reshape(int taskSize)
   {
      selectionMatrix.reshape(taskSize, numberOfDoFs);
      privilegedJointspaceMotion.reshape(taskSize, 1);
      weightMatrix.reshape(taskSize, taskSize);
   }
   
   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   public void setPrivilegedJointspaceMotion(DenseMatrix64F objective)
   {
      privilegedJointspaceMotion.set(objective);
   }

   public void setWeight(DenseMatrix64F weight)
   {
      weightMatrix.set(weight);
   }
}
