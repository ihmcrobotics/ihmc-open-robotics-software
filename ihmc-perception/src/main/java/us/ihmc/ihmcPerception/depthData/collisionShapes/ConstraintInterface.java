package us.ihmc.ihmcPerception.depthData.collisionShapes;

import org.ejml.data.DenseMatrix64F;

public interface ConstraintInterface
{
   /**
    * @return the type of constraint (equality, inequality, strict inequality)
    * Note: All constraints are less than constraints
    */
   public ConstraintType getConstraintType();
   /**
    * Returns the number of variables that are under this constraint
    * @return
    */
   public int getNumberOfConstraintVariables();
   /**
    * 
    * @param xVector the variable vector at which the constraint is to be evaluated
    * @return true or false 
    */
   public boolean isConstraintSatisfied(DenseMatrix64F xVector, double epsilon);
   /**
    * Returns the constraint violation
    * @param xVector
    * @return
    */
   public double getConstraintViolation(DenseMatrix64F xVector);
   /**
    * Returns the constraint Jacobian evaluate at vector specified
    * @param xVector
    * @return
    */
   public DenseMatrix64F getConstraintJacobian(DenseMatrix64F xVector);
}
