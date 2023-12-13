package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;

public interface ConstraintFunction
{
   public double calculate(DMatrixD1 x);
}
