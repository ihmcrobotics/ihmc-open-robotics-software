package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixRMaj;

public interface FunctionOutputCalculator
{
   abstract DMatrixRMaj computeOutput(DMatrixRMaj inputParameter);
}
