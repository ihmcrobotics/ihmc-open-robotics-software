package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;

public interface MeasurementModelElement
{
   public abstract void computeMatrixBlocks();

   public abstract DenseMatrix64F getOutputMatrixBlock(ControlFlowOutputPort<?> statePort);

   public abstract DenseMatrix64F getMeasurementCovarianceMatrixBlock();

   public abstract DenseMatrix64F computeResidual();

   public abstract ArrayList<ControlFlowOutputPort<?>> getStatePorts();
}
