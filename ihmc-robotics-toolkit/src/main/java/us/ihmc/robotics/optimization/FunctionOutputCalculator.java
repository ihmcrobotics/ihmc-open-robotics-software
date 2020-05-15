package us.ihmc.robotics.optimization;

import org.ejml.data.DenseMatrix64F;

public interface FunctionOutputCalculator
{
   abstract DenseMatrix64F computeOutput(DenseMatrix64F inputParameter);
}
