package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;

public class RhoQPInput
{
   private final DenseMatrix64F taskJacobian = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F taskObjective = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F taskWeightMatrix = new DenseMatrix64F(0, 0);

   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public DenseMatrix64F getTaskJacobian()
   {
      return taskJacobian;
   }

   public DenseMatrix64F getTaskObjective()
   {
      return taskObjective;
   }

   public DenseMatrix64F getTaskWeightMatrix()
   {
      return taskWeightMatrix;
   }
}
