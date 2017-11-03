package us.ihmc.ihmcPerception.depthData.collisionShapes;

import java.security.InvalidParameterException;

import org.ejml.data.DenseMatrix64F;

/**
 * Implements an abstract constraint and some generic functions that will be applicable to all types of constraints
 * @author Apoorv
 *
 */
public abstract class AbstractConstraint implements ConstraintInterface
{
   /**
    * Stores the number of variables that constitute the constraint
    */
   protected int n;
   /**
    * Stores the type of constraint (equality, inequality, strict inequality)
    */
   protected ConstraintType type;
   /**
    * Variable for calculation of the constraint violation
    */
   protected DenseMatrix64F constraintViolation;
   protected DenseMatrix64F constraintJacobian;
   
   public AbstractConstraint(int n, ConstraintType type)
   {
      this.n = n;
      this.type = type;
      this.constraintViolation = new DenseMatrix64F(1, 1);
      this.constraintJacobian = new DenseMatrix64F(1, n);
   }

   @Override
   public int getNumberOfConstraintVariables()
   {
      return n;
   }

   @Override
   public ConstraintType getConstraintType()
   {
      return type;
   }

   @Override
   public boolean isConstraintSatisfied(DenseMatrix64F xVector, double epsilon)
   {
      return type.applyMathOperator(getConstraintViolation(xVector), epsilon);
   }

   protected void validateXVector(DenseMatrix64F xVector)
   {
      if (xVector.getNumRows() != n && xVector.getNumCols() == 1)
         throw new InvalidParameterException("Got a state vector of length " + xVector.getNumRows() + ", should have been: " + n);
   }
}
