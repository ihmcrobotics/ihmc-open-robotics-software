package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;

public interface MeasurementModelElement
{
   public abstract void computeMatrixBlocks();

   public abstract DenseMatrix64F getOutputMatrixBlock(ControlFlowOutputPort<?> statePort);

   public abstract DenseMatrix64F getMeasurementCovarianceMatrixBlock();

   public abstract DenseMatrix64F computeResidual();

   public abstract Set<ControlFlowOutputPort<?>> getStatePorts();

   public abstract ControlFlowInputPort<?> getMeasurementPort();
}
