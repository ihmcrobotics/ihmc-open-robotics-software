package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;

public interface CostFunction
{
   public double calculate(DMatrixD1 x);
}
