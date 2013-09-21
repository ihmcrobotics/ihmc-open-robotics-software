package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;

public interface MeasurementModelElement
{
   public abstract void computeMatrixBlocks();

   public abstract DenseMatrix64F getOutputMatrixBlock(ControlFlowOutputPort<?> statePort);

   public abstract DenseMatrix64F getMeasurementCovarianceMatrixBlock();

   public abstract DenseMatrix64F computeResidual();

   public abstract List<ControlFlowOutputPort<?>> getStatePorts();
}
