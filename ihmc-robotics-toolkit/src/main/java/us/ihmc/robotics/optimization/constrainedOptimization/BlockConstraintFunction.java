package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;

public interface BlockConstraintFunction
{
   /**
    * blocks is a list of vectors
    */
   public double calculate(DMatrixD1... blocks);
}
