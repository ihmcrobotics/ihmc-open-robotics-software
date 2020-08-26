package us.ihmc.robotics.optimization;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;

import java.util.function.UnaryOperator;

public interface OutputCalculator extends UnaryOperator<DMatrixRMaj>
{
   default void setIndicesToCompute(TIntArrayList indicesToCompute)
   {}

   default void resetIndicesToCompute()
   {}
}
