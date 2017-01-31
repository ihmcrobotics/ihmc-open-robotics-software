package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;

public abstract class ICPQPInput
{
   public DenseMatrix64F quadraticTerm;
   public DenseMatrix64F linearTerm;
}
