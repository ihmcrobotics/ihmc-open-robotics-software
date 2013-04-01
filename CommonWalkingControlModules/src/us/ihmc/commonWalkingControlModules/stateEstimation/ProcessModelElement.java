package us.ihmc.commonWalkingControlModules.stateEstimation;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;

public interface ProcessModelElement
{
   public abstract void computeMatrixBlocks();

   public abstract DenseMatrix64F getStateMatrixBlock(ControlFlowOutputPort<?> statePort);

   public abstract DenseMatrix64F getInputMatrixBlock(ControlFlowInputPort<?> inputPort);

   public abstract DenseMatrix64F getProcessCovarianceMatrixBlock();

   public abstract void propagateState(double dt);

   public abstract void correctState(DenseMatrix64F correction);

   public abstract TimeDomain getTimeDomain();
}
