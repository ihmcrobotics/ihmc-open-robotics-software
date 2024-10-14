package us.ihmc.robotics.linearAlgebra.careSolvers.schur;

import org.ejml.data.Matrix;
import org.ejml.interfaces.decomposition.DecompositionInterface;

public interface SchurDecomposition<T extends Matrix> extends DecompositionInterface<T>
{
   T getU(T UToPack);

   T getT(T TToPack);
}
